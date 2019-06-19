/* Copyright (c) 2013-2015 Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 * MeshDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DImesh, INDImesh, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreFrustum.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/render_panel.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/validate_floats.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz_textured_quads/mesh_display_custom.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

namespace rviz
{

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.D);
  valid = valid && validateFloats(msg.K);
  valid = valid && validateFloats(msg.R);
  valid = valid && validateFloats(msg.P);
  return valid;
}

MeshDisplayCustom::MeshDisplayCustom()
  : Display()
  , mesh_nodes_(NULL)
  , textures_(NULL)
  , projector_nodes_(NULL)
  , manual_objects_(NULL)
  , decal_frustums_(NULL)
  , new_image_(false)
{
  image_topic_selector_property_ = new RosTopicProperty("Image Topic Selector", "",
      QString::fromStdString(ros::message_traits::datatype<std_msgs::String>()),
      "Image topic selector to subscribe to.",
      this, SLOT(updateDisplayImageTopic()));
  image_topic_property_ = new RosTopicProperty("Image Topic", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic to subscribe to.",
      this, SLOT(updateDisplayImages()));
  // TODO(lucasw) add controls to switch which plane to align to?
  tf_frame_property_ = new TfFrameProperty("Quad Frame", "map",
      "Align the image quad to the xy plane of this tf frame",
      this, 0, true);

  meters_per_pixel_property_ = new FloatProperty("Meters per pixel", 0.002,
      "Rviz meters per image pixel.", this);
}

MeshDisplayCustom::~MeshDisplayCustom()
{
  unsubscribe();
  unsubscribeselector();
  // TODO(lucasw) switch to smart pointers
  delete manual_objects_;
  delete decal_frustums_;
  delete textures_;
  delete mesh_nodes_;

  for (size_t i = 0; i < filter_frustums_.size(); ++i)
    delete filter_frustums_[i];

  // TODO(lucasw) clean up other things
  delete image_topic_selector_property_;
  delete image_topic_property_;
  delete tf_frame_property_;
  delete meters_per_pixel_property_;
}

void MeshDisplayCustom::onInitialize()
{
  tf_frame_property_->setFrameManager(context_->getFrameManager());
  Display::onInitialize();
}

void MeshDisplayCustom::createProjector(int index)
{
  decal_frustums_ = new Ogre::Frustum();

  projector_nodes_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  projector_nodes_->attachObject(decal_frustums_);

  Ogre::SceneNode* filter_node;

  // back filter
  for (size_t i = 0; i < filter_frustums_.size(); ++i)
    delete filter_frustums_[i];
  filter_frustums_.push_back(new Ogre::Frustum());
  filter_frustums_.back()->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  filter_node = projector_nodes_->createChildSceneNode();
  filter_node->attachObject(filter_frustums_.back());
  filter_node->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y));
}

void MeshDisplayCustom::addDecalToMaterial(int index, const Ogre::String& matName)
{
  Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(matName);
  mat->setCullingMode(Ogre::CULL_NONE);
  Ogre::Pass* pass = mat->getTechnique(0)->createPass();

  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthBias(1);
  // pass->setLightingEnabled(true);

  // need the decal_filter to avoid back projection
  Ogre::String resource_group_name = "decal_textures_folder";
  Ogre::ResourceGroupManager& resource_manager = Ogre::ResourceGroupManager::getSingleton();
  if (!resource_manager.resourceGroupExists(resource_group_name))
  {
    resource_manager.createResourceGroup(resource_group_name);
    resource_manager.addResourceLocation(ros::package::getPath("rviz_textured_quads") +
        "/tests/textures/", "FileSystem", resource_group_name, false);
    resource_manager.initialiseResourceGroup(resource_group_name);
  }
  // loads files into our resource manager
  resource_manager.loadResourceGroup(resource_group_name);

  Ogre::TextureUnitState* tex_state = pass->createTextureUnitState();  // "Decal.png");
  tex_state->setTextureName(textures_->getTexture()->getName());
  tex_state->setProjectiveTexturing(true, decal_frustums_);

  tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);
  tex_state->setColourOperation(Ogre::LBO_REPLACE);  // don't accept additional effects

  for (int i = 0; i < filter_frustums_.size(); i++)
  {
    tex_state = pass->createTextureUnitState("Decal_filter.png");
    tex_state->setProjectiveTexturing(true, filter_frustums_[i]);
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::TFO_NONE);
  }
}

shape_msgs::Mesh MeshDisplayCustom::constructMesh(geometry_msgs::Pose mesh_origin,
    float width, float height, float border_size)
{
  shape_msgs::Mesh mesh;

  Eigen::Affine3d trans_mat;
  tf::poseMsgToEigen(mesh_origin, trans_mat);

  // Rviz Coordinate System: x-right, y-forward, z-down
  // create mesh vertices and tranform them to the specified pose

  Eigen::Vector4d top_left(-width / 2.0f - border_size, 0.0f, -height / 2.0f - border_size, 1.0f);
  Eigen::Vector4d top_right(width / 2.0f + border_size, 0.0f, -height / 2.0f - border_size, 1.0f);
  Eigen::Vector4d bottom_left(-width / 2.0f - border_size, 0.0f, height / 2.0f + border_size, 1.0f);
  Eigen::Vector4d bottom_right(width / 2.0f + border_size, 0.0f, height / 2.0f + border_size, 1.0f);

  Eigen::Vector4d trans_top_left = trans_mat.matrix() * top_left;
  Eigen::Vector4d trans_top_right = trans_mat.matrix() * top_right;
  Eigen::Vector4d trans_bottom_left = trans_mat.matrix() * bottom_left;
  Eigen::Vector4d trans_bottom_right = trans_mat.matrix() * bottom_right;

  std::vector<geometry_msgs::Point> vertices(4);
  vertices.at(0).x = trans_top_left[0];
  vertices.at(0).y = trans_top_left[1];
  vertices.at(0).z = trans_top_left[2];
  vertices.at(1).x = trans_top_right[0];
  vertices.at(1).y = trans_top_right[1];
  vertices.at(1).z = trans_top_right[2];
  vertices.at(2).x = trans_bottom_left[0];
  vertices.at(2).y = trans_bottom_left[1];
  vertices.at(2).z = trans_bottom_left[2];
  vertices.at(3).x = trans_bottom_right[0];
  vertices.at(3).y = trans_bottom_right[1];
  vertices.at(3).z = trans_bottom_right[2];
  mesh.vertices = vertices;

  std::vector<shape_msgs::MeshTriangle> triangles(2);
  triangles.at(0).vertex_indices[0] = 0;
  triangles.at(0).vertex_indices[1] = 1;
  triangles.at(0).vertex_indices[2] = 2;
  triangles.at(1).vertex_indices[0] = 1;
  triangles.at(1).vertex_indices[1] = 2;
  triangles.at(1).vertex_indices[2] = 3;
  mesh.triangles = triangles;

  return mesh;
}

void MeshDisplayCustom::clearStates()
{
  if (manual_objects_)
    manual_objects_->clear();

  const int num_quads = 1;
  // resize state vectors
  mesh_poses_.resize(num_quads);

  last_meshes_.resize(num_quads);

  last_images_.resize(num_quads);

  border_colors_.resize(4);
  for (size_t i = 0; i < 4; ++i)
  {
    border_colors_[i] = 1.0;
  }
}

void MeshDisplayCustom::constructQuads(const sensor_msgs::Image::ConstPtr& image)
{
  clearStates();

  int q = 0;
  {
    processImage(q, *image);

    geometry_msgs::Pose mesh_origin;

    // TODO(lucasw) get pose from tf
    std::string frame = tf_frame_property_->getFrameStd();
    if (frame == "")
    {
      frame = image->header.frame_id;
    }
    // Lookup transform into fixed frame
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(frame, ros::Time(0),
        position, orientation))
    {
      std::stringstream ss;
      ss << "Error transforming from fixed frame to frame " << frame.c_str();
      throw ss.str();
    }

    mesh_origin.position.x = position[0];
    mesh_origin.position.y = position[1];
    mesh_origin.position.z = position[2];
    mesh_origin.orientation.w = orientation[0];
    mesh_origin.orientation.x = orientation[1];
    mesh_origin.orientation.y = orientation[2];
    mesh_origin.orientation.z = orientation[3];

    // Rotate from x-y to x-z plane:
    Eigen::Affine3d trans_mat;
    tf::poseMsgToEigen(mesh_origin, trans_mat);
    trans_mat = trans_mat * Eigen::Quaterniond(0.70710678, -0.70710678f, 0.0f, 0.0f);

    Eigen::Quaterniond xz_quat(trans_mat.rotation());
    mesh_origin.orientation.x = xz_quat.x();
    mesh_origin.orientation.y = xz_quat.y();
    mesh_origin.orientation.z = xz_quat.z();
    mesh_origin.orientation.w = xz_quat.w();

    const float meters_per_pixel = meters_per_pixel_property_->getFloat();
    float width = 1.0;
    float height = 1.0;
    if (meters_per_pixel > 0)
    {
      width = image->width * meters_per_pixel;
      height = image->height * meters_per_pixel;
    }
    else
    {
      height = width * image->height / image->width;
    }

    // set properties
    mesh_poses_[q] = mesh_origin;
    img_widths_ = image->width;
    img_heights_ = image->height;

    // default border size (no border)
    border_sizes_ = 0.0f;

    shape_msgs::Mesh mesh = constructMesh(mesh_origin, width, height, border_sizes_);

    physical_widths_ = width;
    physical_heights_ = height;

    boost::mutex::scoped_lock lock(mesh_mutex_);

    // create our scenenode and material
    load();

    if (!manual_objects_)
    {
      static uint32_t count = 0;
      std::stringstream ss;
      ss << "MeshObject" << count++ << "Index" << q;
      manual_objects_ = context_->getSceneManager()->createManualObject(ss.str());
      mesh_nodes_->attachObject(manual_objects_);
    }

    // If we have the same number of tris as previously, just update the object
    if (last_meshes_[q].vertices.size() > 0 && mesh.vertices.size() * 2 == last_meshes_[q].vertices.size())
    {
      manual_objects_->beginUpdate(0);
    }
    else  // Otherwise clear it and begin anew
    {
      manual_objects_->clear();
      manual_objects_->estimateVertexCount(mesh.vertices.size() * 2);
      manual_objects_->begin(mesh_materials_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const std::vector<geometry_msgs::Point>& points = mesh.vertices;
    for (size_t i = 0; i < mesh.triangles.size(); i++)
    {
      // make sure we have front-face/back-face triangles
      for (int side = 0; side < 2; side++)
      {
        std::vector<Ogre::Vector3> corners(3);
        for (size_t c = 0; c < 3; c++)
        {
          size_t corner = side ? 2 - c : c;  // order of corners if side == 1
          corners[corner] = Ogre::Vector3(points[mesh.triangles[i].vertex_indices[corner]].x,
              points[mesh.triangles[i].vertex_indices[corner]].y,
              points[mesh.triangles[i].vertex_indices[corner]].z);
        }
        Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
        normal.normalise();

        for (size_t c = 0; c < 3; c++)
        {
          manual_objects_->position(corners[c]);
          manual_objects_->normal(normal);
        }
      }
    }

    manual_objects_->end();

    mesh_materials_->setCullingMode(Ogre::CULL_NONE);

    last_meshes_[q] = mesh;
  }
}

void MeshDisplayCustom::updateImage(const sensor_msgs::Image::ConstPtr& image)
{
  cur_image_ = image;
  new_image_ = true;
}

void MeshDisplayCustom::updateMeshProperties()
{
  {
    // update color/alpha
    Ogre::Technique* technique = mesh_materials_->getTechnique(0);
    Ogre::Pass* pass = technique->getPass(0);

    Ogre::ColourValue self_illumination_color(0.0f, 0.0f, 0.0f, 0.0f);
    pass->setSelfIllumination(self_illumination_color);

    Ogre::ColourValue diffuse_color(0.0f, 0.0f, 0.0f, 1.0f);
    pass->setDiffuse(diffuse_color);

    Ogre::ColourValue ambient_color(border_colors_[0],
        border_colors_[1], border_colors_[2], border_colors_[3]);
    pass->setAmbient(ambient_color);

    Ogre::ColourValue specular_color(0.0f, 0.0f, 0.0f, 1.0f);
    pass->setSpecular(specular_color);

    Ogre::Real shininess = 64.0f;
    pass->setShininess(shininess);

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    context_->queueRender();
  }
}

void MeshDisplayCustom::updateImageTopic(const std_msgs::String& imagetopic)
{
  if (!isEnabled())
  {
    return;
  }

  if (imagetopic.data.c_str())
  {
    try
    {
      image_sub_ = nh_.subscribe(imagetopic.data.c_str(),
                                 1, &MeshDisplayCustom::updateImage, this);
      setStatus(StatusProperty::Ok, "Display Images Topic", "ok");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Display Images Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void MeshDisplayCustom::updateDisplayImageTopic()
{
  unsubscribeselector();
  subscribeselector();
}

void MeshDisplayCustom::subscribeselector()
{
  if (!isEnabled())
  {
    return;
  }

if (!image_topic_selector_property_->getTopic().isEmpty()) {
    try
    {
      image_topic_sub_ = nh_.subscribe(image_topic_selector_property_->getTopicStd(),
          1, &MeshDisplayCustom::updateImageTopic, this);
      setStatus(StatusProperty::Ok, "Image Topic Selector", "ok");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Image Topic Selector", QString("Error subscribing: ") + e.what());
    }
  }
}

void MeshDisplayCustom::unsubscribeselector()
{
  image_topic_sub_.shutdown();
}

void MeshDisplayCustom::updateDisplayImages()
{
  unsubscribe();
  subscribe();
}

void MeshDisplayCustom::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (image_topic_property_->getTopic() == "transparent") {
    image_transparent_property_ = 1;
  } else if (!image_topic_property_->getTopic().isEmpty()) {
    image_transparent_property_ = 0;
    try
    {
      image_sub_ = nh_.subscribe(image_topic_property_->getTopicStd(),
          1, &MeshDisplayCustom::updateImage, this);
      setStatus(StatusProperty::Ok, "Display Images Topic", "ok");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Display Images Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void MeshDisplayCustom::unsubscribe()
{
  image_sub_.shutdown();
}

void MeshDisplayCustom::load()
{
  if (mesh_nodes_ != NULL)
    return;

  static int count = 0;
  std::stringstream ss;
  ss << "MeshNode" << count++;
  Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();
  Ogre::String resource_group_name =  ss.str();

  Ogre::ResourceGroupManager& rg_mgr = Ogre::ResourceGroupManager::getSingleton();

  Ogre::String material_name = resource_group_name + "MeshMaterial";

  if (!rg_mgr.resourceGroupExists(resource_group_name))
  {
    rg_mgr.createResourceGroup(resource_group_name);

    mesh_materials_ = material_manager.create(material_name, resource_group_name);
    Ogre::Technique* technique = mesh_materials_->getTechnique(0);
    Ogre::Pass* pass = technique->getPass(0);

    Ogre::ColourValue self_illumnation_color(0.0f, 0.0f, 0.0f, border_colors_[3]);
    pass->setSelfIllumination(self_illumnation_color);

    Ogre::ColourValue diffuse_color(border_colors_[0],
        border_colors_[1], border_colors_[2], border_colors_[3]);
    pass->setDiffuse(diffuse_color);

    Ogre::ColourValue ambient_color(border_colors_[0],
        border_colors_[1], border_colors_[2], border_colors_[3]);
    pass->setAmbient(ambient_color);

    Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
    pass->setSpecular(specular_color);

    Ogre::Real shininess = 64.0f;
    pass->setShininess(shininess);

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mesh_materials_->setCullingMode(Ogre::CULL_NONE);
  }

  mesh_nodes_ = this->scene_node_->createChildSceneNode();
}

void MeshDisplayCustom::onEnable()
{
  subscribe();
}

void MeshDisplayCustom::onDisable()
{
  unsubscribe();
}

void MeshDisplayCustom::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void MeshDisplayCustom::update(float wall_dt, float ros_dt)
{
  if (cur_image_)
  {
    // need to run these every frame in case tf has changed,
    // but could detect that.
    try
    {
      constructQuads(cur_image_);
    }
    catch (std::string& e)
    {
      setStatus(StatusProperty::Error, "Display Image", e.c_str());
      return;
    }
    catch (cv_bridge::Exception& e)
    {
      setStatus(StatusProperty::Error, "Display Image", e.what());
      return;
    }
    updateMeshProperties();
    // TODO(lucasw) do what is necessary for new image, but separate
    // other stuff.
    new_image_ = false;
  }

  if (textures_ && !image_topic_property_->getTopic().isEmpty())
  {
    try
    {
      updateCamera(textures_->update());
    }
    catch (UnsupportedImageEncoding& e)
    {
      setStatus(StatusProperty::Error, "Display Image", e.what());
      return;
    }
    catch (std::string& e)
    {
      setStatus(StatusProperty::Error, "Display Image", e.c_str());
      return;
    }
  }
  setStatus(StatusProperty::Ok, "Display Image", "ok");
}

void MeshDisplayCustom::updateCamera(bool update_image)
{
  int index = 0;
  if (update_image)
  {
    last_images_[index] = textures_->getImage();
  }

  if (!img_heights_ || !img_widths_ ||
      !physical_widths_ || !physical_heights_ ||
      !last_images_[index])
  {
    throw("sizes or image not ready");
    // return false;
  }

  boost::mutex::scoped_lock lock(mesh_mutex_);

  float img_width  = img_widths_;
  float img_height = img_heights_;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(last_images_[index]->header.frame_id,
      last_images_[index]->header.stamp, position, orientation);

  Eigen::Affine3d trans_mat;
  tf::poseMsgToEigen(mesh_poses_[index], trans_mat);

  // Rotate by 90 deg to get xz plane
  trans_mat = trans_mat * Eigen::Quaterniond(0.70710678, -0.70710678f, 0.0f, 0.0f);

  float z_offset = (img_width > img_height) ? img_width : img_height;
  float scale_factor = 1.0f /
      ((physical_widths_ > physical_heights_) ?
       physical_widths_ : physical_heights_);

  Eigen::Vector4d projector_origin(0.0f, 0.0f, 1.0f / (z_offset * scale_factor), 1.0f);
  Eigen::Vector4d projector_point = trans_mat.matrix() * projector_origin;

  position = Ogre::Vector3(projector_point[0], projector_point[1], projector_point[2]);
  orientation = Ogre::Quaternion(mesh_poses_[index].orientation.w,
      mesh_poses_[index].orientation.x, mesh_poses_[index].orientation.y,
      mesh_poses_[index].orientation.z);

  // Update orientation with 90 deg offset (xy to xz)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_Z);


  // std::cout << "CameraInfo dimensions: " << last_info_->width << " x " << last_info_->height << std::endl;
  // std::cout << "Texture dimensions: " << last_image_->width << " x " << last_image_->height << std::endl;
  // std::cout << "Original image dimensions: "
  //    << last_image_->width*full_image_binning_ << " x "
  //    << last_image_->height*full_image_binning_ << std::endl;

  // If the image width/height is 0 due to a malformed caminfo, try to grab the width from the image.
  if (img_width <= 0)
  {
    ROS_ERROR("Malformed CameraInfo on camera [%s], width = 0", qPrintable(getName()));
    // use texture size, but have to remove border from the perspective calculations
    img_width = textures_->getWidth() - 2;
  }

  if (img_height <= 0)
  {
    ROS_ERROR("Malformed CameraInfo on camera [%s], height = 0", qPrintable(getName()));
    // use texture size, but have to remove border from the perspective calculations
    img_height = textures_->getHeight() - 2;
  }

  // if even the texture has 0 size, return
  if (img_height <= 0.0 || img_width <= 0.0)
  {
    std::string text = "Could not determine width/height of image due to malformed ";
    text += "CameraInfo (either width or height is 0) and texture.";
    throw text;
  }

  // projection matrix
  float P[12] =
  {
      1.0, 0.0, img_width / 2.0f, 0.0,
      0.0, 1.0, img_height / 2.0f, 0.0,
      0.0, 0.0, 1.0, 0.0
  };

  // calculate projection matrix
  double fx = P[0];
  double fy = P[5];

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (P[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (P[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if (!validateFloats(position))
  {
    throw "CameraInfo/P resulted in an invalid position calculation (nans or infs)";
  }

  if (projector_nodes_ != NULL)
  {
    projector_nodes_->setPosition(position);
    projector_nodes_->setOrientation(orientation);
  }

  // calculate the projection matrix
  double cx = P[2];
  double cy = P[6];

  double far_plane = 100;
  double near_plane = 0.01;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0] = 2.0 * fx / img_width;
  proj_matrix[1][1] = 2.0 * fy / img_height;

  proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width);
  proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5);

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1;

  if (decal_frustums_ != NULL)
    decal_frustums_->setCustomProjectionMatrix(true, proj_matrix);

  // ROS_INFO(" Camera (%f, %f)", proj_matrix[0][0], proj_matrix[1][1]);
  // ROS_INFO(" Render Panel: %x   Viewport: %x", render_panel_, render_panel_->getViewport());


  setStatus(StatusProperty::Ok, "Time", "ok");
  setStatus(StatusProperty::Ok, "Camera Info", "ok");

  if (mesh_nodes_ != NULL && filter_frustums_.size() == 0 && !mesh_materials_.isNull())
  {
    createProjector(index);

    addDecalToMaterial(index, mesh_materials_->getName());
  }
}

void MeshDisplayCustom::clear()
{
  textures_->clear();

  context_->queueRender();

  setStatus(StatusProperty::Warn, "Image", "No Image received");
}

void MeshDisplayCustom::reset()
{
  Display::reset();
  clear();
}

void MeshDisplayCustom::processImage(int index, const sensor_msgs::Image& msg)
{
  // std::cout<<"camera image received"<<std::endl;
  cv_bridge::CvImagePtr cv_ptr;

  // simply converting every image to RGBA
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);

  // make transparent if subscribed image topic is transparent
  if (image_transparent_property_) {
    for(int i = 0; i < cv_ptr->image.rows; i++)
    {
        for(int j = 0; j < cv_ptr->image.cols; j++)
        {
            cv::Vec4b& pixel = cv_ptr->image.at<cv::Vec4b>(i,j);
            pixel[3] = 0;
        }
    }
  }

  // add completely white transparent border to the image so that it won't 
  // replicate colored pixels all over the mesh
  cv::Scalar value(255, 255, 255, 0);
  cv::copyMakeBorder(cv_ptr->image, cv_ptr->image, 1, 1, 1, 1, cv::BORDER_CONSTANT, value);
  cv::flip(cv_ptr->image, cv_ptr->image, -1);

  // Output modified video stream
  if (textures_ == NULL)
    textures_ = new ROSImageTexture();

  textures_->addMessage(cv_ptr->toImageMsg());
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::MeshDisplayCustom, rviz::Display)
