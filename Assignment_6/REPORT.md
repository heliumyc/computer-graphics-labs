# USAGE

```
mkdir build; cd build;
cmake -DCMAKE_BUILD_TYPE=Release ..;
build;
./RasterViewer
```



# COMMANDS

1. insertion mode

   **press i,** and click three points to connect a triangle

2. Transition mode

   **Press o**

   press the mouse and drag triangle

   or click the triangle and press h to clock-wise rotate, **j to counter-clock-wise rotate, k to scale up 25%, l to scale down 25%** 

3. delete mode

   **Press p,** and click any triangle to delete

4. color mode

   press c, and click any triangle, and press 1-9 to color the nearest vertex

5. view mode

   **press + (shift and =)** to zoom in

   **press -** to zoom out

   press w,a,s,d to move scenes up/left/down/right

6. Animation mode

   **press k to capture a keyframe**

   **press x** to clear **all keyframe**

   **press** n to start playing linear interpolation animation

   press b to start bezier interpolation animation