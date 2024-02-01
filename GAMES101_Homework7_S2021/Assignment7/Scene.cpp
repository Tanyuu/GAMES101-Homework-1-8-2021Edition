//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <assert.h>

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = intersect(ray);
    float eps = 1e-4;

    if(!p.happened) return Vector3f();

    //assert(p.distance>eps);
    if(p.m->hasEmission())
    {
        return p.m->getEmission();
    }

    Vector3f Lo_dir, Lo_indir;

    float light_pdf;
    Intersection light_hit;
    sampleLight(light_hit, light_pdf); // light_hit only with coords, normal, emit!!
    Vector3f p_to_light = (light_hit.coords - p.coords).normalized();
    Intersection tmp = intersect(Ray(p.coords, p_to_light));

    //assert(tmp.happened);
    // WHY NOT HAPPEN HERE? MAYBE the ray vertically hit on a triangular surface, and floating-point error
    // if(!tmp.happened)
    // {
    //     std::cout<<"p: "<<p.coords.x<<' '<<p.coords.y<<' '<<p.coords.z<<std::endl;
    //     std::cout<<"l: "<<light_hit.coords.x<<' '<<light_hit.coords.y<<' '<<light_hit.coords.z<<std::endl;
    //     std::cout<<"t: "<<tmp.coords.x<<' '<<tmp.coords.y<<' '<<tmp.coords.z<<std::endl;
    //     std::cout<<' '<<tmp.distance<<' '<<(tmp.coords-p.coords).norm()<<std::endl;

    //     assert(tmp.happened);
    // }
    //assert(fabs((tmp.coords-p.coords).norm()-tmp.distance)<eps);
    if((tmp.coords-p.coords).norm() >= (light_hit.coords - p.coords).norm() - eps)
    {
        Vector3f f_r = p.m->eval(p_to_light, -ray.direction, p.normal);
        float r2 = dotProduct(light_hit.coords - p.coords, light_hit.coords - p.coords);
        float cosA = std::max(0.f, dotProduct(p.normal, p_to_light));
        float cosB = std::max(0.f, dotProduct(light_hit.normal, -p_to_light));
        Lo_dir = light_hit.emit * f_r * cosA * cosB / r2 / light_pdf;
        /*if(!depth) *///std::cout<<f_r<<' '<< cosA << ' '<<cosB<<' '<<r2<<' '<<light_pdf<<std::endl;
    }

    if(get_random_float() < RussianRoulette)
    {
        Vector3f p_to_q = p.m->sample(-ray.direction, p.normal).normalized();
        float pdf = p.m->pdf(-ray.direction, p_to_q, p.normal);
        if(pdf > eps) // Hint2
        {
            Intersection q = intersect(Ray(p.coords, p_to_q));
            if(q.happened && !q.m->hasEmission())
            {
                Vector3f f_r = p.m->eval(p_to_q, -ray.direction, p.normal);
                float cosA = std::max(0.f, dotProduct(p_to_q, p.normal));
                Lo_indir = castRay(Ray(p.coords, p_to_q), depth+1) * f_r * cosA / pdf / RussianRoulette;
    // if(!depth)std::cout<<(Lo_dir + Lo_indir).x<<' '<<(Lo_dir + Lo_indir).y<<' '<<(Lo_dir + Lo_indir).z<<std::endl;
            }
        }
    }

    return Lo_dir + Lo_indir;
}