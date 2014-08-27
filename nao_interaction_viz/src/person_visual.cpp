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
#include <rviz/ogre_helpers/mesh_shape.h>

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
//  Ogre::Quaternion quat(std::sqrt(2) / 2,  0, std::sqrt(2) / 2, 0);
//  setFrameOrientation(quat);
//  static const Ogre::Quaternion test_orientation = Ogre::Vector3::UNIT_Z.getRotationTo( Ogre::Vector3::UNIT_X );
//  setFrameOrientation(test_orientation);
  object_node_ = frame_node_->createChildSceneNode();
  person_name_node_ = object_node_->createChildSceneNode();
  person_age_node_ = object_node_->createChildSceneNode();

  // Initialize the axes
  axes_head_.reset(new rviz::Axes(scene_manager_, object_node_));
  axes_head_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

  // Initialize the name
  person_age_.reset(new rviz::MovableText("EMPTY"));
  person_age_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  person_age_->setCharacterHeight(0.1);
  person_age_->showOnTop();
  person_age_->setColor(Ogre::ColourValue::Black);
  person_age_->setVisible(true);
  person_age_node_->attachObject(person_age_.get());


  // Initialize the age
  person_name_.reset(new rviz::MovableText("EMPTY"));
  person_name_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  person_name_->setCharacterHeight(0.1);
  person_name_->showOnTop();
  person_name_->setColor(Ogre::ColourValue::Black);
  person_name_->setVisible(true);
  person_name_node_->attachObject(person_name_.get());

  // Initialize the cylinder for a basic person
  person_body_.reset(new rviz::PersonShape(rviz::PersonShape::BODY,
                                           scene_manager,
                                           object_node_));

  person_gender_.reset(new rviz::PersonShape(rviz::PersonShape::GENDER_MALE,
                                             scene_manager_,
                                             object_node_));

  person_valence_.reset(new rviz::PersonShape(rviz::PersonShape::VALENCE_NEUTRAL,
                                              scene_manager_,
                                              object_node_));
}

PersonVisual::~PersonVisual() {
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(object_node_);
  scene_manager_->destroySceneNode(person_name_node_);
  scene_manager_->destroySceneNode(frame_node_);
}

void PersonVisual::setMessage(const nao_interaction_msgs::Person& person, bool do_display_id,
                              bool do_display_confidence, bool do_display_face) {

  Ogre::Quaternion quat(std::sqrt(2) / 2,  0, 0, std::sqrt(2) / 2);

  // BODY
  float body_alpha = 0.8;
  if (person.person_confidence > 0.1)
  {
    body_alpha = person.person_confidence;
  }
  person_body_->setOrientation(quat);
  person_body_->setColor(0.2, 0.2, 0.2, body_alpha);
  Ogre::Vector3 scale(person.width, person.width/2, person.height);
  person_body_->setScale(scale);
  Ogre::Vector3 position(person.pose.x,
                         person.pose.y,
                         0);
  person_body_->setPosition(position);

  // GENDER
//  Ogre::Quaternion image_quat(std::sqrt(2) / 2,  0, std::sqrt(2) / 2, 0);
  Ogre::Quaternion image_quat(0.5,  0.5, 0.5, 0.5);
  rviz::PersonShape::Type gender = rviz::PersonShape::GENDER_FEMALE;
  if (person.gender[0] == person.GENDER_MALE)
  {
    gender = rviz::PersonShape::GENDER_MALE;
  }
  person_gender_->changeMaterial(gender);
  person_gender_->setOrientation(image_quat);
  Ogre::Vector3 scale_gender(0.0007, 0.0007, 1);
  person_gender_->setScale(scale_gender);
  person_gender_->setColor(1.0, 1.0, 1.0, 0.9);
  Ogre::Vector3 gender_position(position.x,
                                position.y-1.5*person.width,
                                person.height);
  person_gender_->setPosition(gender_position);


  // VALENCE
  rviz::PersonShape::Type valence = rviz::PersonShape::VALENCE_LOWEST;
  if (person.emotion.valence[0] > -60)
  {
    valence = rviz::PersonShape::VALENCE_LOW;
  }
  if (person.emotion.valence[0] > -20)
  {
    valence = rviz::PersonShape::VALENCE_NEUTRAL;
  }
  if (person.emotion.valence[0] > 20)
  {
    valence = rviz::PersonShape::VALENCE_HIGH;
  }
  if (person.emotion.valence[0] > 60)
  {
    valence = rviz::PersonShape::VALENCE_HIGHEST;
  }
  // Initialize the valence gender pictogram
  person_valence_->changeMaterial(valence);
  person_valence_->setOrientation(image_quat);
  //  Ogre::Vector3 scale_gender(0.002, 0.002, 1);
  person_valence_->setScale(scale_gender);
  person_valence_->setColor(1.0, 1.0, 1.0, 0.9);
  Ogre::Vector3 valence_position(position.x,
                                 position.y-1.0*person.width,
                                 person.height);
  person_valence_->setPosition(valence_position);


  // NAME/ID
  std::stringstream caption;
  caption << "ID:" << person.id;
  person_name_->setCaption(caption.str());
  person_name_->setVisible(true);
  Ogre::Vector3 name_position(position.x,
                              position.y-1.0*person.width,
                              person.height-0.15);
  person_name_node_->setPosition(name_position);

  // AGE
  std::stringstream age_caption;
  age_caption << person.age[0] << " years";
  person_age_->setCaption(age_caption.str());
  person_age_->setVisible(true);
  Ogre::Vector3 age_position(position.x,
                              position.y-1.45*person.width,
                              person.height-0.25);
  person_age_node_->setPosition(age_position);

  // Deal with the face
  //  if ((!do_display_face) || (face.height == 0.0))
  //    return;
  //  Ogre::Vector3 position_head(0, 0, 0);
  //  axes_head_->setPosition(position_head);
}

// Position and orientation are passed through to the SceneNode.
void PersonVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void PersonVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}
}
