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

#include <fstream>
#include <stdio.h>
#include <string>

#include <boost/foreach.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/frame_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include "person_visual.h"

#include "person_display.h"

#include <nao_interaction_msgs/Person.h>
#include <ros/package.h>


namespace nao_interaction_viz {

PersonDisplay::PersonDisplay() {
  do_display_id_ = new rviz::BoolProperty("ID", false, "Display the DB ID or not.", this);
  do_display_confidence_ = new rviz::BoolProperty("Confidence", true, "Display the match confidence or not.", this);
  do_display_face_ = new rviz::BoolProperty("Face", true, "Display information about the face.", this);
}

class Update : public ros::CallbackInterface
{


public:
  Update(PersonDisplay &pd)
    : person_display_(pd)
  {}

  CallResult call()
  {
    ros::Time now = ros::Time::now();
    if ((now.toSec()-person_display_.old.toSec())>1.0)
    {
    ROS_DEBUG_STREAM("update transparency for person display");
    for (int i=0; i<person_display_.visuals_.size(); ++i)
    {
      person_display_.visuals_[i]->setFrameTransparency(0.1);
    }
    }
    return TryAgain;

  }

private:
  PersonDisplay& person_display_;
};


// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void PersonDisplay::onInitialize() {
  MFDClass::onInitialize();

  // setup a new resource group for the pictograms
  const std::string resource_name = "nao_interaction_viz";
  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resource_name);
  Ogre::ResourceGroupManager::getSingleton().
      addResourceLocation( ros::package::getPath(ROS_PACKAGE_NAME), "FileSystem", resource_name);
  ROS_INFO_STREAM("adding resource group" << ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  context_->getUpdateQueue()->addCallback( ros::CallbackInterfacePtr( new Update(*this) ) );
}

// Clear the visuals by deleting their objects.
void PersonDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

// This is our callback to handle an incoming message.
void PersonDisplay::processMessage(const nao_interaction_msgs::PersonsConstPtr& msg)
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this message. If
  // it fails, we can't do anything else so we return.

  old = ros::Time::now();
  visuals_.clear();
  for (size_t i_msg = 0; i_msg < msg->persons.size(); ++i_msg)
  {
    const nao_interaction_msgs::Person& object = msg->persons[i_msg];
    // Create a new visual for that message
    boost::shared_ptr<PersonVisual> visual = boost::shared_ptr<PersonVisual>(
        new PersonVisual(context_->getSceneManager(), scene_node_, context_));
    visuals_.push_back(visual);

    // Define the visual
    visual->setMessage(object, do_display_id_->getBool(), do_display_confidence_->getBool(),
        do_display_face_->getBool());

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(object.header.frame_id,
                                                   object.header.stamp,
                                                   position,
                                                   orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", object.header.frame_id.c_str(),
          qPrintable(fixed_frame_));
      return;
    }
    ROS_INFO_STREAM("base translation: " << position);

    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);
  }
}
}  // end namespace nao_interaction_viz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nao_interaction_viz::PersonDisplay, rviz::Display)
