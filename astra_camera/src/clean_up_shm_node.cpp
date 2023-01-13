#include <fcntl.h>
#include <semaphore.h>
#include <sys/shm.h>

#include <cstring>
#include <iostream>

#include "astra_camera/constants.h"

int main() {
  using namespace astra_camera;
  sem_t *sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 0);
  if (sem == SEM_FAILED) {
    std::cout << "sem_open failed: " << strerror(errno);
    return 1;
  }
  std::cout << "sem_open succeeded";
  sem_close(sem);
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  std::cout << "sem_unlink succeeded" << std::endl;
  int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (shm_id != -1) {
    shmctl(shm_id, IPC_RMID, nullptr);
    std::cout << "shmctl `IPC_RMID` succeeded" << std::endl;
  }
  return 0;
}
