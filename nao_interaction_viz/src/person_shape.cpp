#include "person_shape.h"
#include <ros/ros.h>
#include <OGRE/OgreEntity.h>

namespace rviz
{
PersonShape::PersonShape(Shape::Type type, Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node):
    Shape(type, scene_manager, parent_node)
{
    ROS_INFO_STREAM("PersonShape called");

    // since createEntity is not virtual .. make a hack
    scene_manager_->destroySceneNode( scene_node_->getName() );
    scene_manager_->destroySceneNode( offset_node_->getName() );
    scene_manager_->destroyEntity( entity_ );

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "Shape" << count++;
    entity_ = this->createEntity(ss.str(), type, scene_manager);

    scene_node_ = parent_node->createChildSceneNode();
    offset_node_ = scene_node_->createChildSceneNode();
    offset_node_->attachObject( entity_ );
    entity_->setMaterialName(material_name_);

#if (OGRE_VERSION_MAJOR <= 1 && OGRE_VERSION_MINOR <= 4)
    entity_->setNormaliseNormals(true);
#endif
}

Ogre::Entity* PersonShape::createEntity(const std::string &name, Shape::Type type, Ogre::SceneManager *scene_manager)
{
    ROS_INFO_STREAM("my entity is called");
    // ignore shape_type since this has to be a person
    std::string mesh_name = "ninja.mesh";
    return scene_manager->createEntity(name, mesh_name);
}

}
