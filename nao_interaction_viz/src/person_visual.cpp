/*
 * Copyright (C) 2014 Aldebaran Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>
#include <sstream>

#include <OGRE/OgreCommon.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreVector3.h>

#include <rviz/display_context.h>
#include <rviz/display_factory.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "person_visual.h"
#include "person_shape.h"

namespace nao_interaction_viz {

// BEGIN_TUTORIAL
PersonVisual::PersonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
    rviz::DisplayContext* display_context) :
    display_context_(display_context) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
  object_node_ = frame_node_->createChildSceneNode();

  // Initialize the axes
  axes_head_.reset(new rviz::Axes(scene_manager_, object_node_));
  axes_head_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

  // Initialize the name
  name_.reset(new rviz::MovableText("EMPTY"));
  name_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  name_->setCharacterHeight(0.08);
  name_->showOnTop();
  name_->setColor(Ogre::ColourValue::White);
  name_->setVisible(false);

  // Initialize the cylinder
//  cylinder_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager, object_node_));
  cylinder_.reset(new rviz::PersonShape(rviz::Shape::Cylinder, scene_manager, object_node_));

  object_node_->attachObject(name_.get());
}

PersonVisual::~PersonVisual() {
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(object_node_);
  scene_manager_->destroySceneNode(frame_node_);
}

void PersonVisual::setMessage(const nao_interaction_msgs::Person& person, bool do_display_id,
    bool do_display_confidence, bool do_display_face) {
  Ogre::Vector3 position(person.pose.x, person.pose.y, 0);

  // Set the name of the object
  std::stringstream caption;
  if ((!person.id.empty()) && (do_display_id))
    caption << person.id << std::endl;

  // Set the caption of the object
  if (do_display_confidence)
    caption << person.confidence;

  if (caption.str().empty())
    name_->setVisible(false);
  else {
    name_->setCaption(caption.str());
    name_->setVisible(true);
    name_->setLocalTranslation(Ogre::Vector3(0, person.height, 0));
  }

  const nao_interaction_msgs::Face &face = person.face;

  // Deal with the cylinder display
//  cylinder_->setColor(1.0, 0.0, 0.0, 0.8);
  // Deal with the height of the cylinder
  float height = person.height/10;
  if (face.height != 0.0)
    // Make sure the cylinder is a bit shorter to display the head
    height = face.height;
  position.z = height / 2;
  object_node_->setPosition(position);
  Ogre::Quaternion quat(std::sqrt(2) / 2, std::sqrt(2) / 2, 0, 0);
  cylinder_->setOrientation(quat);
  Ogre::Vector3 scale(person.width, height, person.width);
  cylinder_->setScale(scale);

  // Deal with the face
  if ((!do_display_face) || (face.height == 0.0))
    return;
  Ogre::Vector3 position_head(0, face.height, 0);
  axes_head_->setPosition(position_head);
}

// Position and orientation are passed through to the SceneNode.
void PersonVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void PersonVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}
}
