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

#ifndef NAO_INTERACTION_DISPLAY_H
#define NAO_INTERACTION_DISPLAY_H

#include <vector>

#include <rviz/message_filter_display.h>

#include <nao_interaction_msgs/Persons.h>

namespace nao_interaction_viz {

class PersonVisual;

// PersonDisplay will display a person perceived by NAO
class PersonDisplay: public rviz::MessageFilterDisplay<nao_interaction_msgs::Persons> {
  Q_OBJECT
public:

  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  PersonDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void
  onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void
  reset();

  // Function to handle an incoming ROS message.
private:
  void
  processMessage(const nao_interaction_msgs::PersonsConstPtr& msg);

  /** Storage for the list of visuals */
  std::vector<boost::shared_ptr<PersonVisual> > visuals_;
  /** flag indicating whether we display the id of a person */
  rviz::BoolProperty* do_display_id_;
  /** flag indicating whether the confidence should be displayed */
  rviz::BoolProperty* do_display_confidence_;
  /** flag indicating whether information about the face should be displayed */
  rviz::BoolProperty* do_display_face_;

  friend class Update;
  ros::Time old;

};

}

#endif // NAO_INTERACTION_DISPLAY_H
