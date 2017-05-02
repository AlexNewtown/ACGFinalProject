#ifndef _PHOTON_H_
#define _PHOTON_H_

#include <glm/glm.hpp>


// ===========================================================
// Class to store the information when a photon hits a surface

class Photon {
 public:

  // CONSTRUCTOR
  Photon(const glm::vec3 &p, const glm::vec3 &d, const glm::vec3 &e, const glm::vec3 &n, int b) :
    position(p),direction_from(d),energy(e),normal(n),bounce(b) {num=1;}

  void setEnergy(glm::vec3 e) {energy=e;num++;}
  void setPosition(glm::vec3 p) {position=p;}

  // ACCESSORS
  const glm::vec3& getPosition() const { return position; }
  const glm::vec3& getDirectionFrom() const { return direction_from; }
  const glm::vec3& getEnergy() const { return energy; }
  const glm::vec3& getNormal() const {return normal; }
  int whichBounce() const { return bounce; }
  int getNum() const {return num;}

 private:
  // REPRESENTATION
  glm::vec3 position;
  glm::vec3 direction_from;
  glm::vec3 energy;
  glm::vec3 normal;
  int bounce,num;
};

#endif
