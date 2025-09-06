//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f();
    }

    // 打到光源
    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }

    Vector3f l_dir(0,0,0);
    Vector3f l_indir(0,0,0);
    switch(inter.m->getType()){
        case DIFFUSE:{
            // 对光源积分
            Intersection lightInter;
            float pdf_light = 0.0f;

            sampleLight(lightInter, pdf_light);

            Vector3f ws = lightInter.coords - inter.coords;
            Vector3f ws_n = ws.normalized();
            float power = ws.x * ws.x + ws.y * ws.y + ws.z * ws.z;

            Ray shadowRay(inter.coords, ws_n);
            Intersection t = intersect(shadowRay);
            if (t.distance - ws.norm() > -EPSILON)
            {
                l_dir = lightInter.emit * inter.m->eval(ray.direction, ws_n, inter.normal) 
                    * dotProduct(ws_n, inter.normal) 
                    * dotProduct(-ws_n, lightInter.normal) 
                    / power / pdf_light;
            }

            if (get_random_float() > RussianRoulette) {
                return l_dir;
            }

            // 对其他方向积分
            Vector3f wi = inter.m->sample(ray.direction, inter.normal).normalized();
            Ray newRay(inter.coords, wi);
            Intersection nextInter = intersect(newRay);
            if (nextInter.happened && !nextInter.m->hasEmission())
            {
                float pdf = inter.m->pdf(ray.direction, wi, inter.normal);
                if (pdf > EPSILON)
                {
                    l_indir = castRay(newRay, depth + 1) 
                        * inter.m->eval(ray.direction, wi, inter.normal) 
                        * dotProduct(wi, inter.normal)
                        / pdf / RussianRoulette;
                }
            }           
            break;
        }
        case MIRROR:{
            if (get_random_float() > RussianRoulette) {
                return l_dir;
            }
            Vector3f wi = inter.m->sample(ray.direction, inter.normal).normalized();
            Ray newRay(inter.coords, wi);
            Intersection nextInter = intersect(newRay);
            if (nextInter.happened)
            {
                float pdf = inter.m->pdf(ray.direction, wi, inter.normal);
                if (pdf > EPSILON)
                {
                    l_indir = castRay(newRay, depth + 1) 
                        * inter.m->eval(ray.direction, wi, inter.normal) 
                        * dotProduct(wi, inter.normal)
                        / pdf / RussianRoulette;
                }
            }
            break;
        }
    }
    return l_dir + l_indir;
}
/**/