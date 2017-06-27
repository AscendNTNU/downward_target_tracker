#define TESTING_WITH_LAPTOP 1 // I set this to 1 to test stuff on my laptop's webcam

#if TESTING_WITH_LAPTOP==1
#define camera_project m_project_pinhole
#define camera_inverse_project m_ray_pinhole
#else
#define camera_project m_project_equidistant
#define camera_inverse_project m_ray_equidistant
#endif
