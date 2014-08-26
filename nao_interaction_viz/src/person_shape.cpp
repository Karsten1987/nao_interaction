#include "person_shape.h"
#include <ros/ros.h>
#include <OGRE/OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <rviz/ogre_helpers/stl_loader.h>
#include <OgreResourceGroupManager.h>
namespace rviz
{
PersonShape::PersonShape(PersonShape::Type type, Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node):
  Object(scene_manager),
  type_(type)
{
  ROS_INFO_STREAM("PersonShape called");

  // TAKE THE NAME INTO ACCOUNT !!!
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "Shape" << count++;
  this->createEntity(ss.str(), type, scene_manager);

  scene_node_ = parent_node->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  offset_node_->attachObject( entity_ );
  //    entity_->setMaterialName(material_name_);

#if (OGRE_VERSION_MAJOR <= 1 && OGRE_VERSION_MINOR <= 4)
  entity_->setNormaliseNormals(true);
#endif
}

PersonShape::~PersonShape()
{
  scene_manager_->destroySceneNode(scene_node_->getName() );
  scene_manager_->destroySceneNode(offset_node_->getName() );

  if (entity_)
    scene_manager_->destroyEntity( entity_ );

  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}


void PersonShape::createEntity(const std::string &name,
                               PersonShape::Type type,
                               Ogre::SceneManager *scene_manager)
{
  ROS_INFO_STREAM("my entity is called");

  ogre_tools::STLLoader stl_loader;
  Ogre::MeshPtr mesh_ptr;
  std::string mesh_name;

  switch (type)
  {
  case FACE:
    mesh_name = "rviz_person_face";
    createMaterial("textures/face_neutral.png");
    entity_ = scene_manager->createEntity(name,  Ogre::SceneManager::PT_PLANE);
    break;

  case BODY:
    createMaterial();
    mesh_name = "rviz_person_body";
    stl_loader.load("/home/kknese/rviz_body.stl");
    mesh_ptr = stl_loader.toMesh(name);
    entity_ = scene_manager->createEntity(name, mesh_ptr->getName() );
    break;

  case GENDER_MALE:
    mesh_name = "rviz_person_gender_male";
    createMaterial("textures/person_gender_male.png");
    entity_ = scene_manager->createEntity(name,  Ogre::SceneManager::PT_PLANE);
    break;
  }
  entity_->setMaterial(material_);
}

bool PersonShape::createMaterial( const std::string& image_name,
                                  const std::string& resource_group)
{
  if (image_name == "default")
  {
    material_ = Ogre::MaterialManager::getSingleton().create( "default",
                                                              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    material_->setCullingMode(Ogre::CULL_NONE);
    return true;
  }
  material_ = Ogre::MaterialManager::getSingleton().load( image_name, resource_group );
  material_->setCullingMode(Ogre::CULL_NONE);
  if( material_.isNull() )
    return false;

  Ogre::Pass* first_pass = material_->getTechnique(0)->getPass(0);

  if( first_pass->getNumTextureUnitStates() == 0 )
  {
    Ogre::TextureUnitState* texture_unit = first_pass->createTextureUnitState();
    Ogre::TexturePtr texture
        = Ogre::TextureManager::getSingleton().load( image_name, resource_group );
    if( texture.isNull() )
      return false;

    texture_unit->setTexture( texture ); // or setTextureName if Ogre 1.8?
  }
  return true;
}


void PersonShape::setColor(const Ogre::ColourValue& c)
{
  material_->getTechnique(0)->setAmbient( c * 0.5 );
  material_->getTechnique(0)->setDiffuse( c );
  if ( c.a < 0.9998 )
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    material_->getTechnique(0)->setDepthWriteEnabled( true );
  }
}

void PersonShape::setColor( float r, float g, float b, float a )
{
  setColor(Ogre::ColourValue(r, g, b, a));
}

void PersonShape::setOffset( const Ogre::Vector3& offset )
{
  offset_node_->setPosition( offset );
}

void PersonShape::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void PersonShape::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void PersonShape::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( scale );
}

const Ogre::Vector3& PersonShape::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& PersonShape::getOrientation()
{
  return scene_node_->getOrientation();
}

void PersonShape::setUserData( const Ogre::Any& data )
{
  if (entity_)
    entity_->setUserAny( data );
  else
    ROS_ERROR("Shape not yet fully constructed. Cannot set user data. Did you add triangles to the mesh already?");
}



//bool PersonShape::loadImage(const Ogre::String& texture_name, const Ogre::String& texture_path)
//{
//  bool image_loaded = false;
//  std::ifstream ifs(texture_path.c_str(), std::ios::binary|std::ios::in);
//  if (ifs.is_open())
//  {
//    Ogre::String tex_ext;
//    Ogre::String::size_type index_of_extension = texture_path.find_last_of('.');
//    if (index_of_extension != Ogre::String::npos)
//    {
//      tex_ext = texture_path.substr(index_of_extension+1);
//      Ogre::DataStreamPtr data_stream(new Ogre::FileStreamDataStream(texture_path, &ifs, false));
//      Ogre::Image img;
//      img.load(data_stream, tex_ext);
//      Ogre::TextureManager::getSingleton().loadImage(texture_name,
//                                                     Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, img, Ogre::TEX_TYPE_2D, 0, 1.0f);
//      image_loaded = true;
//    }
//    ifs.close();
//  }
//  return image_loaded;
//}



}
