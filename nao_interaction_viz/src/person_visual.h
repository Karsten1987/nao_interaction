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

#ifndef NAO_INTERACTION_VIZ_H
#define NAO_INTERACTION_VIZ_H

#include <string>

#include <nao_interaction_msgs/Person.h>

namespace Ogre {
class Entity;
class Quaternion;
class SceneManager;
class SceneNode;
class Vector3;
}

namespace rviz {
class Axes;
class DisplayContext;
class MovableText;
class Shape;
}

namespace nao_interaction_viz {
// Declare the visual class for this display.
// Each instance of PersonVisual represents a person
class PersonVisual {
public:
  /** Constructor.  Creates the visual stuff and puts it into the
   * scene, but in an unconfigured state.
   *
   * @param display The display that calls those visuals
   */
  PersonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz::DisplayContext* display);

  // Destructor.  Removes the visual stuff from the scene.
  virtual
  ~PersonVisual();

  /** Configure the visual to show the data in the message.
   * @param msg the message defining the object to add a visual for
   * @param name the name of the object to display
   * @param mesh_file The file in which the mesh is stored
   * @param do_display_id whether the object id is displayed
   * @param do_display_confidence whether the object confidence is displayed
   * @param do_display_face whether information about the face is displayed
   */
  void
  setMessage(const nao_interaction_msgs::Person& msg, bool do_display_id, bool do_display_confidence,
      bool do_display_face);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way ImuVisual is only
  // responsible for visualization.
  void
  setFramePosition(const Ogre::Vector3& position);
  void
  setFrameOrientation(const Ogre::Quaternion& orientation);

  /**
   * Set the mesh of the visual by using a file and setting it to the mesh resource
   * @param mesh_file The file in which the mesh is stored
   */
  void
  setMesh(const std::string& mesh_file);
private:
  rviz::DisplayContext* display_context_;

  /** The name of the person */
  boost::shared_ptr<rviz::MovableText> name_;

  /** The pose of the object */
  boost::shared_ptr<rviz::Axes> axes_head_;

  /** The cylinder representing the person */
  boost::shared_ptr<rviz::Shape> cylinder_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Object message header.
  Ogre::SceneNode* frame_node_;
  Ogre::SceneNode* object_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

}    // end namespace rviz_plugin_tutorials

#endif // NAO_INTERACTION_VIZ_H
