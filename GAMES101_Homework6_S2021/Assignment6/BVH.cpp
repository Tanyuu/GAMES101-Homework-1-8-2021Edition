#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    static bool SAH = true;

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
            });
            break;
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();
        if(SAH)
        {
            Bounds3 preBounds[objects.size()], sufBounds[objects.size()];
            for(int i=0;i<objects.size();i++)
            {
                preBounds[i] = Union(objects[i]->getBounds(), i-1>=0?preBounds[i-1]:Bounds3());
            }
            for(int i=objects.size()-1;i>=0;i--)
            {
                if(i+1<objects.size())
                {
                    sufBounds[i] = Union(objects[i+1]->getBounds(), i+2<objects.size()?sufBounds[i+2]:Bounds3());
                }
            }
            float cost = INFINITY;
            int icost = -1;
            for(int i=1;i<objects.size()-1;i++)
            {
                float costTmp = ((i+1)*preBounds[i].SurfaceArea()+(objects.size()-i-1)*sufBounds[i].SurfaceArea())/bounds.SurfaceArea();
                if(costTmp < cost)
                {
                    cost = costTmp;
                    icost = i;
                }
            }
            middling = objects.begin() + icost + 1;
        }


        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    if(node->bounds.IntersectP(
        ray, Vector3f{1.0/ray.direction.x,1.0/ray.direction.y,1.0/ray.direction.z}, 
        std::array<int, 3>{(int)(ray.direction.x>0), (int)(ray.direction.y>0), (int)(ray.direction.z>0)}))
        {
            if(node->left!=nullptr&&node->right!=nullptr)
            {
                Intersection tmp1 = getIntersection(node->left, ray);
                Intersection tmp2 = getIntersection(node->right, ray);
                //assert(tmp1.happened || tmp2.happened);
                if(tmp1.happened && tmp2.happened)
                {
                    if(tmp1.distance < tmp2.distance)
                        inter = tmp1;
                    else
                        inter = tmp2;
                }
                else if(tmp1.happened)
                {
                    inter = tmp1;
                }
                else if(tmp2.happened)
                {
                    inter = tmp2;
                }
            }
            else
            {
                inter = node->object->getIntersection(ray);
            }
        }
    return inter;
}