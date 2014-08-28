#include <rviz/ogre_helpers/object.h>
#include <OGRE/OgreSceneManager.h>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
class Entity;
}

namespace rviz
{

/*
 * This class is basically a 100 procent copy of rviz::shape
 * however this was necessary since all the createEntity functions are static
 * if not it would be super hacky
 */
class PersonShape : public Object
{
public:

  enum Type
  {
    BODY = 10,
    FACE = 11,
    GENDER_MALE = 0,
    GENDER_FEMALE = 1,
    VALENCE_HIGHEST = 4,
    VALENCE_HIGH = 5,
    VALENCE_NEUTRAL = 6,
    VALENCE_LOW = 7,
    VALENCE_LOWEST = 8,
  };

  PersonShape(Type type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = NULL);
  virtual ~PersonShape();

  Type getType() { return type_; }

  void setOffset( const Ogre::Vector3&  offset );

  virtual void setColor( float r, float g, float b, float a );
  void setColor( const Ogre::ColourValue& c );
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setScale( const Ogre::Vector3& scale );
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

  Ogre::SceneNode* getRootNode() { return scene_node_; }  void setUserData( const Ogre::Any& data );
  Ogre::Entity* getEntity() { return entity_; }
  Ogre::MaterialPtr getMaterial() { return material_; }

  bool createMaterial(const std::string& image_name = "default",
                      const std::string& resource_group = "nao_interaction_viz"/*"Autodetect"*/ );

  bool changeMaterial(const PersonShape::Type type);

  // overwrite createEntity process and fill in a mesh
  void createEntity(const std::string& name,
                             PersonShape::Type shape_type,
                             Ogre::SceneManager* scene_manager);


  //  static bool loadImage(const Ogre::String& texture_name, const Ogre::String& texture_path);

protected:
  void setMaterial();

  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;
  Ogre::Entity* entity_;
  Ogre::MaterialPtr material_;
  Type type_;
  const std::string mesh_file_;

private:
  static const std::string& getMeshRoot();
};

} // namespace rviz
