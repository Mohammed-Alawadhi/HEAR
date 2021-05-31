// Version: 3.0
// Author: Mohamad Chehadeh
// Last Modification By: Mohamad Wahbah
// Date: 27 Jan 2020
//Revision Note: Added assignment operator and constructor with init_lists, added  the "==" and "!=" logic operators

#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP

template  <class L> class Vector2D;
#include <cstdint>
#include <math.h>
#include <initializer_list>

template <class T>  
class Vector3D {
        public:
                T x = 0;
                T y = 0;
                T z = 0;
                Vector3D<T>(){};
                Vector3D<T>(std::initializer_list<T> tmp){this->x = *tmp.begin(); this->y = *(tmp.begin()+1); this->z = *(tmp.begin()+2);};

                Vector3D<T> operator + (Vector3D<T> obj) {
                        Vector3D<T> res;
                        res.x = x + obj.x;
                        res.y = y + obj.y;
                        res.z = z + obj.z;
                        return res;
                }
                Vector3D<T> operator - (Vector3D<T> obj) {
                        Vector3D<T> res;
                        res.x = x - obj.x;
                        res.y = y - obj.y;
                        res.z = z - obj.z;
                        return res;
                }
                Vector3D<T> operator * (Vector3D<T> obj) {
                        Vector3D<T> res;
                        res.x = x * obj.x;
                        res.y = y * obj.y;
                        res.z = z * obj.z;
                        return res;
                }
                Vector3D<T> operator * (T tmp) {
                        Vector3D<T> res;
                        res.x = this->x * tmp;
                        res.y = this->y * tmp;
                        res.z = this->z * tmp;
                        return res;
                }
                Vector3D<T> operator / (T tmp) {
                        Vector3D<T> res;
                        res.x = this->x / tmp;
                        res.y = this->y / tmp;
                        res.z = this->z / tmp;
                        return res;
                }
                void operator = (T tmp) {
                        this->x = tmp;
                        this->y = tmp;
                        this->z = tmp;
                }
                void operator = (std::initializer_list<T> tmp) {
                        this->x = *tmp.begin();
                        this->y = *(tmp.begin()+1);
                        this->z = *(tmp.begin()+2);
                }
                bool operator == (Vector3D<T> tmp) {
                        if(this->x == tmp.x && this->y == tmp.y && this->z == tmp.z) {return true;}
                        else {return false;}
                }
                bool operator != (Vector3D<T> tmp) {
                        if(this->x != tmp.x || this->y != tmp.y || this->z != tmp.z) {return true;}
                        else {return false;}
                }
                template <typename M>
                operator Vector3D<M>() {
                        Vector3D<M> tmp;
                        tmp.x = M(this->x);
                        tmp.y = M(this->y);
                        tmp.z = M(this->z);

                        return tmp;
                }
                double operator ^ (Vector3D<T> tmp) {
                        return x*tmp.x + y*tmp.y + z*tmp.z;
                }
                static double getL2Norm(Vector3D<T> vec1) {
                    return sqrt(vec1.x*vec1.x+vec1.y*vec1.y+vec1.z*vec1.z);
                }


                static Vector3D<T> cross(Vector3D<T> vec1,Vector3D<T> vec2) {
                        Vector3D<T> results;
                        results.x=vec1.y*vec2.z-vec1.z*vec2.y;
                        results.y=vec1.z*vec2.x-vec1.x*vec2.z;
                        results.z=vec1.x*vec2.y-vec1.y*vec2.x;
                        return results;
                }

                static double dot(Vector3D<T> vec1,Vector3D<T> vec2) {
                    return vec1.x*vec2.x+vec1.y*vec2.y+vec1.z*vec2.z;
                }

                Vector2D<T> project_xy() {
                        Vector2D<T> res;
                        res.x=this->x;
                        res.y=this->y;
                        return res;
                }
};

#endif