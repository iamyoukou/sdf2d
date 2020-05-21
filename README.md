# sdf2d
Create 2D signed distance field (SDF) using points and polygons.

The accuracy is higher than that of [8ssedt](https://github.com/Lisapple/8SSEDT) method.

Inspired by Yuanming Hu's [Taichi](https://github.com/yuanming-hu/taichi).

## FYI
I refine the original algorithm and reduce a lot of computing time.
You can find the refined version in [this project](https://github.com/iamyoukou/sdf2dWithMPM2D).

# Result
Throwing track (like the one in CSGO)

![result](csgo_throwing_track.gif)
