#include <rviz/ogre_helpers/shape.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{


class PersonShape : public Shape
{
public:

    enum Type
    {
        Person,
    };

    PersonShape(Shape::Type type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

    // overwrite createEntity process and fill in a mesh
    static Ogre::Entity* createEntity(const std::string& name, Shape::Type shape_type, Ogre::SceneManager* scene_manager);

private:

    const std::string mesh_file_;
};
}
