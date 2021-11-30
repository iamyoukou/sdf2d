# sdf2d

Create 2D signed distance field (SDF) using points and polygons.

The accuracy is higher than that of [8ssedt](https://github.com/Lisapple/8SSEDT) method.

Inspired by Yuanming Hu's [Taichi](https://github.com/yuanming-hu/taichi).

![result](throwingTrack.gif)

# License

The MIT License (MIT)

Copyright (c) 2021, Jiang Ye

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Computing gradient

When computing the gradient for a given point,
I use Yuanming Hu's strategy because it performs more smoothly than mine.

![gradient](./res/gradient.png)

The final gradient is `grad = lerp(grad1, grad2, α)` for `x` and `y` direction, respectively.

It means, if the point is close to the left (or bottom) border of the current cell, i.e. `α -> 1`,
we get more gradient of the current cell.
Otherwise, we get more gradient of the `x + 1` (or `y + 1`) cell.

# Reducing computing time

When deal with dynamic objects in a simulating program,
recomputing SDF is necessary.
In this project, I used an expensive one.
But I have refined it in [another project](https://github.com/iamyoukou/sdf2dWithMPM2D).
