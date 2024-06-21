#include <fcntl.h>
#include <ros/ros.h>
#include <semaphore.h>
#include <sys/shm.h>

#include <cstring>
#include <iostream>

#include "astra_camera/constants.h"

int main() {
  using namespace astra_camera;
  sem_t *sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 0);
  if (sem == SEM_FAILED) {
    ROS_ERROR_STREAM("sem_open failed: " << strerror(errno));
    return 1;
  }
  ROS_INFO_STREAM("sem_open succeeded");
  sem_close(sem);
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  ROS_INFO_STREAM("sem_unlink succeeded");
  int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (shm_id != -1) {
    shmctl(shm_id, IPC_RMID, nullptr);
    ROS_INFO_STREAM("shmctl `IPC_RMID` succeeded");
  }
  return 0;
}
