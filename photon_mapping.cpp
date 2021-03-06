#include "glCanvas.h"

#include <iostream>
#include <algorithm>
#include <queue>
#include <glm/gtx/string_cast.hpp>

#include "argparser.h"
#include "photon_mapping.h"
#include "mesh.h"
#include "face.h"
#include "primitive.h"
#include "kdtree.h"
#include "utils.h"
#include "raytracer.h"


// ==========
// DESTRUCTOR
PhotonMapping::~PhotonMapping() {
  // cleanup all the photons
  delete kdtree;
  delete irradianceCache;
}


// ========================================================================
// Recursively trace a single photon

void PhotonMapping::TracePhoton(const glm::vec3 &position, const glm::vec3 &direction,
				const glm::vec3 &energy, int iter, bool caustic) {

    Hit hit;
    bool intersect = raytracer->CastRay(Ray(position+(0.0001*direction),direction),hit,false);
    if (!intersect)
      return;

    glm::vec3 newEnergy, newDirection, newPosition, normal;

    Material* mat = hit.getMaterial();
    normal = hit.getNormal();
    newPosition = hit.getT()*direction+position;

    //Store in kdtree
    if (iter>0 || glm::length(energy) == 0) {
      if (caustic)
        kdtree->AddPhoton(Photon(newPosition,direction,energy,normal,iter));
      else {
        // BoundingBox query;
        // std::vector<Photon> tmp, close;
        // float dist = .05;
        // int num_close_photons = 10;
        // query.Set(newPosition-glm::vec3(dist),glm::vec3(dist)+newPosition);
        // irradianceCache->CollectPhotonsInBox(query,tmp);
        // for (int i=0;i<tmp.size();i++)
        //   if (glm::length(newPosition-tmp[i].getPosition())<dist && glm::length(newEnergy-tmp[i].getEnergy())<EPSILON && glm::length(newEnergy-tmp[i].getEnergy())<EPSILON)
        //     close.push_back(tmp[i]);
        // if (close.size() < num_close_photons)
          irradianceCache->AddPhoton(Photon(newPosition,direction,energy,normal,iter));
        // else if (iter > 0) {
        //   for (int i=0;i<close.size();i++) {
        //     if (glm::length(newPosition-close[i].getPosition()) > EPSILON) {
        //       glm::vec3 interpolatedEnergy = close[i].getEnergy();
        //       interpolatedEnergy += newEnergy/(num_close_photons*square(glm::length(newPosition-close[i].getPosition())));
        //       close[i].setEnergy(interpolatedEnergy);
        //       irradianceCache->UpdatePhoton(close[i]);
        //     }
        //   }
        // }
      }
    }
    //Shadow Photons
    if (iter == 0)
      TracePhoton(newPosition+(0.0001*direction), direction, glm::vec3(0.0,0.0,0.0), iter,false);

    //Reflective
    newEnergy = energy*mat->getReflectiveColor();
    newDirection = MirrorDirection(normal,direction);
    if (glm::length(newEnergy) > EPSILON/100) {
      TracePhoton(newPosition,newDirection,newEnergy,iter+1,true);
    }

    //Diffuse
    newEnergy = energy*mat->getDiffuseColor();
    newDirection = RandomDiffuseDirection(normal);
    if (glm::length(newEnergy) > EPSILON/100) {
      TracePhoton(newPosition,newDirection,newEnergy,iter+1,false);
    }

}


// ========================================================================
// Trace the specified number of photons through the scene

void PhotonMapping::TracePhotons() {
  std::cout << "trace photons" << std::endl;

  // first, throw away any existing photons
  delete kdtree;
  delete irradianceCache;

  // consruct a kdtree to store the photons
  BoundingBox *bb = mesh->getBoundingBox();
  glm::vec3 min = bb->getMin();
  glm::vec3 max = bb->getMax();
  glm::vec3 diff = max-min;
  min -= 0.001f*diff;
  max += 0.001f*diff;
  kdtree = new KDTree(BoundingBox(min,max));
  irradianceCache = new KDTree(BoundingBox(min,max));

  // photons emanate from the light sources
  const std::vector<Face*>& lights = mesh->getLights();

  // compute the total area of the lights
  float total_lights_area = 0;
  for (unsigned int i = 0; i < lights.size(); i++) {
    total_lights_area += lights[i]->getArea();
  }

  // shoot a constant number of photons per unit area of light source
  // (alternatively, this could be based on the total energy of each light)
  for (unsigned int i = 0; i < lights.size(); i++) {
    float my_area = lights[i]->getArea();
    int num = args->num_photons_to_shoot * my_area / total_lights_area;
    // the initial energy for this photon
    glm::vec3 energy = my_area/float(num) * lights[i]->getMaterial()->getEmittedColor();
    glm::vec3 normal = lights[i]->computeNormal();
    std::queue<std::thread> threadQueue;
    std::cout<<std::thread::hardware_concurrency()<<std::endl;
    for (int j = 0; j < num; j++) {
      glm::vec3 start = lights[i]->RandomPoint();
      // the initial direction for this photon (for diffuse light sources)
      glm::vec3 direction = RandomDiffuseDirection(normal);
      while (threadQueue.size() > args->num_threads) {
        if (threadQueue.front().joinable()) {
          threadQueue.front().join();
          threadQueue.pop();
        }
      }
      threadQueue.push(std::thread(&PhotonMapping::TracePhoton,this,start+(0.0001*direction),(direction),(energy),0,false));
      //TracePhoton(start+(0.0001*direction),(direction),(energy),0,false);
    }
  }
}


// ======================================================================

// helper function
bool closest_photon(const std::pair<Photon,float> &a, const std::pair<Photon,float> &b) {
  return (a.second < b.second);
}

void PhotonMapping::GatherPhotons(const glm::vec3 &point, const glm::vec3 &normal, const glm::vec3 &direction_from, std::vector<Photon> &photons, double &radius) const {

  if (kdtree == NULL) {
    std::cout << "WARNING: Photons have not been traced throughout the scene." << std::endl;
    return;
  }

  do {
    radius *= 1.5;
    photons.clear();
    glm::vec3 min = glm::vec3(point.x - radius, point.y - radius, point.z - radius);
    glm::vec3 max = glm::vec3(point.x + radius, point.y + radius, point.z + radius);
    BoundingBox bb(min, max);
    std::vector<Photon> bb_photons;
    kdtree->CollectPhotonsInBox(bb, bb_photons);
    irradianceCache->CollectPhotonsInBox(bb,bb_photons);
    for (unsigned int i = 0; i < bb_photons.size(); ++i) {
      if (fabs(glm::distance(point, bb_photons[i].getPosition()) < radius)) {
        photons.push_back(bb_photons[i]);
      }
    }
  } while (photons.size() < (unsigned)args->num_photons_to_collect && 1.0f/square(radius) > EPSILON);
  //} while (false);

}

bool PhotonMapping::ShadowCounts(const std::vector<Photon> &photons, unsigned int &count_direct, unsigned int &count_shadow) const {

  count_direct = 0;
  count_shadow = 0;
  for (unsigned int i = 0; i < photons.size(); ++i) {
    if (photons[i].whichBounce() == 0) {
      if (glm::length(photons[i].getEnergy()) == 0.0) {
        //Shadow photon
        count_shadow++;
      }
      else {
        //Direct photon
        count_direct++;
      }
    }
  }
}

// ======================================================================
glm::vec3 PhotonMapping::GatherIndirect(const glm::vec3 &point, const glm::vec3 &normal, const glm::vec3 &direction_from, const std::vector<Photon> &photons, double radius) const {

  // photons with energy == 0 and bounce == 0 are shadow photons
  // photons with energy >  0 and bounce == 0 are illumination photons

  BoundingBox query = BoundingBox(point);
  glm::vec3 answer = glm::vec3(0);

  for (int i=0;i<photons.size();i++) {
    if (photons[i].whichBounce() != 0)
      answer += photons[i].getEnergy();
  }

  answer /= float(M_PI*square(radius));
  return answer;
}


// ======================================================================
// PHOTON VISUALIZATION FOR DEBUGGING
// ======================================================================

void PhotonMapping::initializeVBOs() {
  HandleGLError("enter photonmapping initializevbos()");
  glGenBuffers(1, &photon_direction_verts_VBO);
  glGenBuffers(1, &photon_direction_indices_VBO);
  glGenBuffers(1, &kdtree_verts_VBO);
  glGenBuffers(1, &kdtree_edge_indices_VBO);
  HandleGLError("leave photonmapping initializevbos()");
}

void PhotonMapping::setupVBOs() {
  HandleGLError("enter photonmapping setupvbos()");

  photon_direction_verts.clear();
  photon_direction_indices.clear();
  kdtree_verts.clear();
  kdtree_edge_indices.clear();

  // initialize the data
  BoundingBox *bb = mesh->getBoundingBox();
  float max_dim = bb->maxDim();

  if (kdtree == NULL) return;
  std::vector<const KDTree*> todo;
  todo.push_back(kdtree);
  todo.push_back(irradianceCache);
  while (!todo.empty()) {
    const KDTree *node = todo.back();
    todo.pop_back();
    if (node->isLeaf()) {

      // initialize photon direction vbo
      const std::vector<Photon> &photons = node->getPhotons();
      int num_photons = photons.size();
      for (int i = 0; i < num_photons; i++) {
	const Photon &p = photons[i];
	glm::vec3 energy = p.getEnergy()*float(args->num_photons_to_shoot);
        glm::vec4 color(energy.x,energy.y,energy.z,1);
        if (glm::length(energy) == 0.0) color = glm::vec4(0.0,0.0,1.0,1.0);//render shadow photons blue
	const glm::vec3 &position = p.getPosition();
	glm::vec3 other = position - p.getDirectionFrom()*0.02f*max_dim;
        addEdgeGeometry(photon_direction_verts,photon_direction_indices,
                        position,other,color,color,max_dim*0.0005f,0);
      }

      // initialize kdtree vbo
      float thickness = 0.001*max_dim;
      glm::vec3 A = node->getMin();
      glm::vec3 B = node->getMax();
      glm::vec4 black(1,0,0,1);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,A.y,A.z),glm::vec3(A.x,A.y,B.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,A.y,B.z),glm::vec3(A.x,B.y,B.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,B.y,B.z),glm::vec3(A.x,B.y,A.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,B.y,A.z),glm::vec3(A.x,A.y,A.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(B.x,A.y,A.z),glm::vec3(B.x,A.y,B.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(B.x,A.y,B.z),glm::vec3(B.x,B.y,B.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(B.x,B.y,B.z),glm::vec3(B.x,B.y,A.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(B.x,B.y,A.z),glm::vec3(B.x,A.y,A.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,A.y,A.z),glm::vec3(B.x,A.y,A.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,A.y,B.z),glm::vec3(B.x,A.y,B.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,B.y,B.z),glm::vec3(B.x,B.y,B.z),black,black,thickness,thickness);
      addEdgeGeometry(kdtree_verts,kdtree_edge_indices,glm::vec3(A.x,B.y,A.z),glm::vec3(B.x,B.y,A.z),black,black,thickness,thickness);

    } else {
      todo.push_back(node->getChild1());
      todo.push_back(node->getChild2());
    }
  }



  // copy the data to each VBO
  if (photon_direction_verts.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,photon_direction_verts_VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(VBOPosNormalColor) * photon_direction_verts.size(),
                 &photon_direction_verts[0],
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,photon_direction_indices_VBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(VBOIndexedTri) * photon_direction_indices.size(),
                 &photon_direction_indices[0], GL_STATIC_DRAW);
  }
  if (kdtree_verts.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,kdtree_verts_VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(VBOPosNormalColor) * kdtree_verts.size(),
                 &kdtree_verts[0],
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,kdtree_edge_indices_VBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(VBOIndexedTri) * kdtree_edge_indices.size(),
                 &kdtree_edge_indices[0], GL_STATIC_DRAW);
  }

  HandleGLError("leave photonmapping setupvbos()");
}

void PhotonMapping::drawVBOs() {
  HandleGLError("enter photonmapping drawvbos()");

  glUniform1i(GLCanvas::colormodeID, 1);
  if (args->render_photons && photon_direction_verts.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,photon_direction_verts_VBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,photon_direction_indices_VBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)sizeof(glm::vec3) );
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2 + sizeof(glm::vec4)));
    glDrawElements(GL_TRIANGLES,
                   photon_direction_indices.size()*3,
                   GL_UNSIGNED_INT, 0);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
  }

  if (args->render_kdtree && kdtree_edge_indices.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,kdtree_verts_VBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,kdtree_edge_indices_VBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor),(void*)sizeof(glm::vec3) );
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT,GL_FALSE,sizeof(VBOPosNormalColor), (void*)(sizeof(glm::vec3)*2 + sizeof(glm::vec4)));
    glDrawElements(GL_TRIANGLES,
                   kdtree_edge_indices.size()*3,
                   GL_UNSIGNED_INT, 0);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
  }

  HandleGLError("leave photonmapping drawvbos()");
}

void PhotonMapping::cleanupVBOs() {
  glDeleteBuffers(1, &photon_direction_verts_VBO);
  glDeleteBuffers(1, &photon_direction_indices_VBO);
  glDeleteBuffers(1, &kdtree_verts_VBO);
  glDeleteBuffers(1, &kdtree_edge_indices_VBO);
}
