//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const { return this->bvh->Intersect(ray); }

void Scene::sampleLight(Intersection &pos, float &pdf) const {
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index, Object **hitObject) {
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

// TODO: Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
    Intersection itsct = intersect(ray);
    if (itsct.happened)
        return shade(itsct, ray);
    else
        return this->backgroundColor;
}

Vector3f Scene::shade(const Intersection &itsct, const Ray &ray) const {
    static const double eps = 5e-4;
    Material *m = itsct.m;
    Vector3f N = itsct.normal;

    Vector3f L = {0.f, 0.f, 0.f};
    // at light
    if (m->hasEmission())
        L = m->getEmission();
    else {
        Vector3f L_indir(0.f);
        Vector3f L_dir(0.f);

        // to light
        float l_pdf = 0;
        Intersection l_coords;
        sampleLight(l_coords, l_pdf);

        Vector3f ws = (l_coords.coords - itsct.coords).normalized();
        bool notBlocked = (intersect(Ray(itsct.coords, ws)).coords - l_coords.coords).norm() < eps;
        bool hasDirectLight = false;

        // !intersect(Ray(itsct.coords, ws)).happened
        if (notBlocked) {
            Vector3f toLight = l_coords.coords - itsct.coords;
            L_dir = 
                l_coords.emit 
                * m->eval(ws, -ray.direction, N) 
                * std::max(0.0f, dotProduct(ws, N)) 
                * std::max(0.0f, dotProduct(-ws, l_coords.normal)) 
                / dotProduct(toLight, toLight) 
                / l_pdf;
            hasDirectLight = L_dir.norm() > 0.1;
        }
        // to diffuse
        if (get_random_float() <= RussianRoulette) {
            Vector3f wi = m->sample(ray.direction, N);
            Ray toNext(itsct.coords, wi);
            Intersection s_coords = intersect(toNext);
            //  && (!hasDirectLight || !s_coords.m->hasEmission())
            if (s_coords.happened && (!hasDirectLight || !s_coords.m->hasEmission())) {
                L_indir = 
                    shade(s_coords, toNext) 
                    * m->eval(wi, -ray.direction, N) 
                    * std::max(0.0f, dotProduct(wi, N)) 
                    / std::max(m->pdf(wi, -ray.direction, N), 0.0001f) 
                    / RussianRoulette;
            }
        }
        // L = L_dir + L_indir;
        L = Vector3f::Max(Vector3f::Min(L_dir + L_indir, Vector3f(1.0f)), Vector3f(0.0f));
    }
    return L;
}
