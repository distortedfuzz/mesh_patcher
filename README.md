# mesh_patcher
A basic mesh patcher that patches each area that is formed by the intersections of quadruples of geodesic paths. The geodesic paths are formed between the given number of samples, calculated by the Farthest Point Sampling method. There are also 3 different implementations of Dijkstra's algorithm with array, heap and Fibonacci heap. Previously trid different and more basic patching strategies are left in the code, but the project mainly uses a modified version of the Coon's patch by forming Catmull-Rom splines between vertices. A GUI element is added for the user to see all the formed patches with the provided sample counts in the mesh.

This project uses Fiboheap by beniz and Polyscope.run.

The project's blog post for further and in depth information:
https://medium.com/me/stats/post/591040c1e646

Some basic demos:
https://www.youtube.com/watch?v=aO0O1Zah-ac
https://www.youtube.com/watch?v=-vzolp_x7mE

## Dijkstra's Algorithm

![dijkstra_man](https://github.com/user-attachments/assets/5d28fdb0-d174-48a6-96be-c0a77fc8a061)
![dijkstra_man_path](https://github.com/user-attachments/assets/110f4fc4-d1f7-4150-b57c-ac60662c9e66)
![dijkstra_centaur](https://github.com/user-attachments/assets/09818aeb-3dfd-49d8-8b1d-48ae8bf0ccbc)
![dijkstra_centaur_path2](https://github.com/user-attachments/assets/7c0b93f8-2aaf-476c-bc87-d1d45c3dd0b0)
![dijkstra_centaur_path1](https://github.com/user-attachments/assets/a30b78c9-3e48-4b1f-90ce-39523199554f)

## FPS

![fps_man_6](https://github.com/user-attachments/assets/8e4f891f-ae01-49c0-8e46-b8bad692db13)
![fps_man_6_back](https://github.com/user-attachments/assets/a0326421-7b8f-42db-ba88-703fde200fb0)
![fps_centaur_9](https://github.com/user-attachments/assets/7a6ef266-07ed-4737-847b-29b6dd34ee5b)
![fps_centaur_9_back](https://github.com/user-attachments/assets/28b7dbc8-2fa2-4ec5-b5a9-09a4c7fd1d5b)

## Geodesics Between Samples
![geodesics_between_samples_man](https://github.com/user-attachments/assets/ec6184f6-1e67-4361-a95d-2660b196bb94)

## Patching
![centaur_simple_patch2](https://github.com/user-attachments/assets/7c2b89f2-2a9c-4094-ae0f-1bbd51c8c0ee)
![centaur_simple_patch1](https://github.com/user-attachments/assets/16123832-d223-4260-b749-878fdf835d3f)
![basic_patch2](https://github.com/user-attachments/assets/88108539-173f-4f19-856d-76d27b95f1bf)
![basic_patch1](https://github.com/user-attachments/assets/42432027-7192-46ad-81e6-81e0e8b35abe)
