/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#include "astra_camera/utils.h"

// typedef struct OBCameraParams
//{
//   float l_intr_p[4];//[fx,fy,cx,cy]
//   float r_intr_p[4];//[fx,fy,cx,cy]
//   float r2l_r[9];//[r00,r01,r02;r10,r11,r12;r20,r21,r22]
//   float r2l_t[3];//[t1,t2,t3]
//   float l_k[5];//[k1,k2,p1,p2,k3]
//   float r_k[5];
//   //int is_mirror;
// }OBCameraParams;
namespace astra_wrapper {
std::ostream& operator<<(std::ostream& os, const OBCameraParams& params) {
  os << "===left===" << std::endl;
  os << "fx : " << params.l_intr_p[0] << std::endl;
  os << "fy : " << params.l_intr_p[1] << std::endl;
  os << "cx : " << params.l_intr_p[2] << std::endl;
  os << "cy : " << params.l_intr_p[3] << std::endl;
  os << "===end====" << std::endl;
  os << "===right===" << std::endl;
  os << "fx : " << params.r_intr_p[0] << std::endl;
  os << "fy : " << params.r_intr_p[1] << std::endl;
  os << "cx : " << params.r_intr_p[2] << std::endl;
  os << "cy : " << params.r_intr_p[3] << std::endl;
  os << "===end===" << std::endl;
  return os;
}

}  // namespace astra_wrapper
