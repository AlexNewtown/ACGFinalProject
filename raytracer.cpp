#include "glCanvas.h"

#include "raytracer.h"
#include "material.h"
#include "argparser.h"
#include "raytree.h"
#include "utils.h"
#include "mesh.h"
#include "face.h"
#include "primitive.h"
#include "photon_mapping.h"

#include <glm/gtx/rotate_vector.hpp>

// ===========================================================================
// casts a single ray through the scene geometry and finds the closest hit
bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches) const {
  bool answer = false;

  // intersect each of the quads
  for (int i = 0; i < mesh->numOriginalQuads(); i++) {
    Face *f = mesh->getOriginalQuad(i);
    if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
  }
  for (int i = 0; i < mesh->numOriginalTris(); i++) {
      Face *f = mesh->getOriginalTri(i);
      if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
  }

  // intersect each of the primitives (either the patches, or the original primitives)
  if (use_rasterized_patches) {
    for (int i = 0; i < mesh->numRasterizedPrimitiveFaces(); i++) {
      Face *f = mesh->getRasterizedPrimitiveFace(i);
      if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
    }
  } else {
    int num_primitives = mesh->numPrimitives();
    for (int i = 0; i < num_primitives; i++) {
      if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
    }
  }
  return answer;
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
glm::vec3 RayTracer::TraceRay(Ray &ray, Hit &hit, int bounce_count) {

  // First cast a ray and see if we hit anything.
  hit = Hit();
  bool intersect = CastRay(ray,hit,false);

  // if there is no intersection, simply return the background color
  if (intersect == false) {
    return glm::vec3(srgb_to_linear(mesh->background_color.r),
                     srgb_to_linear(mesh->background_color.g),
                     srgb_to_linear(mesh->background_color.b));
  }

  // otherwise decide what to do based on the material
  Material *m = hit.getMaterial();
  assert (m != NULL);

  // rays coming from the light source are set to white, don't bother to ray trace further.
  if (glm::length(m->getEmittedColor()) > 0.001) {
    return glm::vec3(1,1,1);
  }


  glm::vec3 normal = hit.getNormal();
  glm::vec3 point = ray.pointAtParameter(hit.getT());
  glm::vec3 answer;

  bool directly_illuminated = false;
  bool completely_shadowed = false;
  // ----------------------------------------------
  //  start with the indirect light (ambient light)
  glm::vec3 diffuse_color = m->getDiffuseColor(hit.get_s(),hit.get_t());
  if (args->gather_indirect) {
    // photon mapping for more accurate indirect light
    std::vector<Photon> photons;
    double radius = 0.05;
    photon_mapping->GatherPhotons(point,normal,ray.getDirection(),photons,radius);
    answer = diffuse_color * (photon_mapping->GatherIndirect(point,normal,ray.getDirection(),photons,radius) + args->ambient_light);

    unsigned int count_direct = 0;
    unsigned int count_shadow = 0;
    photon_mapping->ShadowCounts(photons, count_direct, count_shadow);
    if (count_shadow == 0) directly_illuminated = true;
    else if (count_direct == 0) completely_shadowed = true;
  } else {
    // the usual ray tracing hack for indirect light
    answer = diffuse_color * args->ambient_light;
  }

  // ----------------------------------------------
  // add contributions from each light that is not in shadow
  int num_lights = mesh->getLights().size();
  if (glm::length(m->getDiffuseColor(hit.get_s(),hit.get_t())) > 0.0) {
    for (int i = 0; i < num_lights; i++) {

      Face *f = mesh->getLights()[i];
      glm::vec3 lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
      glm::vec3 myLightColor;
      glm::vec3 lightCentroid = f->computeCentroid();
      glm::vec3 dirToLightCentroid = glm::normalize(lightCentroid-point);
      float distToLightCentroid = glm::length(lightCentroid-point);

      if (args->num_shadow_samples == 0 || directly_illuminated)
        answer += m->Shade(ray,hit,dirToLightCentroid,lightColor/float(M_PI*distToLightCentroid*distToLightCentroid),args);
      else if (completely_shadowed) {}
      else if (args->num_shadow_samples == 1) {
        Hit shadHit = Hit();
        Ray toLight = Ray(point,dirToLightCentroid);
        RayTree::AddShadowSegment(toLight,0,hit.getT());
        bool rayhit = CastRay(toLight,shadHit,false);
        shadow_rays++;
        myLightColor = lightColor / float(M_PI*distToLightCentroid*distToLightCentroid);
        if (rayhit && shadHit.getMaterial() == f->getMaterial())
          answer += m->Shade(ray,hit,dirToLightCentroid,myLightColor,args);
      } else {
        //answer += m->Shade(ray,hit,dirToLightCentroid,glm::vec3(0,0,1),args); // TESTING blue
        int gridSize = (int)sqrt(args->num_shadow_samples);
        int gridSquare = square(gridSize);
        for (int j=0;j<gridSize;j++) {
          for (int k=0;k<gridSize;k++) {
            glm::vec3 randPoint = f->RandomPoint(gridSize,j,k); // jitter in here
            glm::vec3 dirToRandPoint = glm::normalize(randPoint-point);
            Hit shadHit;
            Ray toLight = Ray(point,dirToRandPoint);
            bool rayhit = CastRay(toLight,shadHit,false);
            shadow_rays++;
            RayTree::AddShadowSegment(toLight,0,shadHit.getT());
            float distToRandPoint = glm::length(randPoint-point);
            myLightColor = lightColor / float(M_PI*pow(distToRandPoint,2));
            myLightColor /= float(args->num_shadow_samples);
            if (rayhit && shadHit.getMaterial() == f->getMaterial())
              answer += m->Shade(ray,hit,dirToRandPoint,myLightColor,args);
          }
        }
        // remainder after square jittering
        for (int j=0;j<args->num_shadow_samples-gridSquare;j++) {
          glm::vec3 randPoint = f->RandomPoint();
          glm::vec3 dirToRandPoint = glm::normalize(randPoint-point);
          Hit shadHit;
          Ray toLight = Ray(point,dirToRandPoint);
          bool rayhit = CastRay(toLight,shadHit,false);
          shadow_rays++;
          RayTree::AddShadowSegment(toLight,0,shadHit.getT());
          float distToRandPoint = glm::length(randPoint-point);
          myLightColor = lightColor / float(M_PI*pow(distToRandPoint,2));
          myLightColor /= float(args->num_shadow_samples);
          if (rayhit && shadHit.getMaterial() == f->getMaterial())
            answer += m->Shade(ray,hit,dirToRandPoint,myLightColor,args);
        }
      }
    }
  }

  // ----------------------------------------------
  // add contribution from reflection, if the surface is shiny
  glm::vec3 reflectiveColor = m->getReflectiveColor();//percentage from reflected light
  if (bounce_count > 0 && glm::length(reflectiveColor) > EPSILON) {
    glm::vec3 V = ray.getDirection();
    glm::vec3 R = glm::normalize(V - (2*glm::dot(V,normal)*normal));
    double roughness = m->getRoughness();

    glm::vec3 glossy_color(0.0,0.0,0.0);
    if (args->num_glossy_samples < 2 || roughness == 0.0) { // mirror reflectance
      Ray reflected(point+(0.0001*normal), R);
      Hit reflected_hit;
      glossy_color = TraceRay(reflected, reflected_hit, bounce_count-1);
      RayTree::AddReflectedSegment(reflected, 0.0, reflected_hit.getT());
    }
    else { // glossy reflectance
      for (int i = 0; i < args->num_glossy_samples; ++i) {
        double rx = R[0]+(args->rand()-0.5)*roughness;
        double ry = R[1]+(args->rand()-0.5)*roughness;
        double rz = R[2]+(args->rand()-0.5)*roughness;
        glm::vec3 d = glm::normalize(glm::vec3(rx,ry,rz));
        Ray glossy_ray(point+(0.0001*normal),d);
        Hit glossy_hit;
        glossy_color += TraceRay(glossy_ray, glossy_hit, bounce_count-1);
        RayTree::AddReflectedSegment(glossy_ray, 0.0, glossy_hit.getT());
      }
      glossy_color /= args->num_glossy_samples;
    }

    answer += reflectiveColor*glossy_color;
  }


  return answer;

}



void RayTracer::initializeVBOs() {
  glGenBuffers(1, &pixels_a_VBO);
  glGenBuffers(1, &pixels_b_VBO);
  glGenBuffers(1, &pixels_indices_a_VBO);
  glGenBuffers(1, &pixels_indices_b_VBO);
  render_to_a = true;
}


void RayTracer::resetVBOs() {

  pixels_a.clear();
  pixels_b.clear();

  pixels_indices_a.clear();
  pixels_indices_b.clear();

  render_to_a = true;
}

void RayTracer::setupVBOs() {

  glBindBuffer(GL_ARRAY_BUFFER,pixels_a_VBO);
  glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPosNormalColor)*pixels_a.size(),&pixels_a[0],GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER,pixels_b_VBO);
  glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPosNormalColor)*pixels_b.size(),&pixels_b[0],GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,pixels_indices_a_VBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
	       sizeof(VBOIndexedTri) * pixels_indices_a.size(),
	       &pixels_indices_a[0], GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,pixels_indices_b_VBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
	       sizeof(VBOIndexedTri) * pixels_indices_b.size(),
	       &pixels_indices_b[0], GL_STATIC_DRAW);

}

void RayTracer::drawVBOs() {
  // turn off lighting
  glUniform1i(GLCanvas::colormodeID, 0);
  // turn off depth buffer
  glDisable(GL_DEPTH_TEST);

  if (render_to_a) {
    drawVBOs_b();
    drawVBOs_a();
  } else {
    drawVBOs_a();
    drawVBOs_b();
  }

  glEnable(GL_DEPTH_TEST);
}

void RayTracer::drawVBOs_a() {
  if (pixels_a.size() == 0) return;
  glBindBuffer(GL_ARRAY_BUFFER, pixels_a_VBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,pixels_indices_a_VBO);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)sizeof(glm::vec3) );
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2));
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2 + sizeof(glm::vec4)));
  glDrawElements(GL_TRIANGLES,
                 pixels_indices_a.size()*3,
                 GL_UNSIGNED_INT, 0);
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
  glDisableVertexAttribArray(3);
}

void RayTracer::drawVBOs_b() {
  if (pixels_b.size() == 0) return;
  glBindBuffer(GL_ARRAY_BUFFER, pixels_b_VBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,pixels_indices_b_VBO);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)sizeof(glm::vec3) );
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2));
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2 + sizeof(glm::vec4)));
  glDrawElements(GL_TRIANGLES,
                 pixels_indices_b.size()*3,
                 GL_UNSIGNED_INT, 0);
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
  glDisableVertexAttribArray(3);
}


void RayTracer::cleanupVBOs() {
  glDeleteBuffers(1, &pixels_a_VBO);
  glDeleteBuffers(1, &pixels_b_VBO);
}
