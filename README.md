# ThorVG

ThorVG是一个开源图形库，专为创建基于矢量的场景和动画而设计。它兼具强大的功能和出色的轻量级效率，因为“Thor”一词具有双重含义——既象征着雷鸣般的强大力量，又象征着闪电般的敏捷。秉承“简单即更好”的理念，ThorVG项目提供了直观、用户友好的界面，同时保持了紧凑的占用空间和最小的开销。 

## 构建安装步骤

```bash
meson setup builddir -Dloaders="all" -Dsavers="all" -Dexamples=false -Dlog="false" --default-library=static --prefix=$(pwd)/install  
ninja -C builddir install
```

## 集成

将/install目录下的头文件与动态库拷贝至集成目录即可

## 使用示例

ThorVG renders vector shapes to a given canvas buffer. The following is a quick start to show you how to use the essential APIs.

First, you should initialize the ThorVG engine:

```cpp
tvg::Initializer::init(4);   //4 threads
```

Then it would be best if you prepared an empty canvas for drawing on it:

```cpp
static uint32_t buffer[WIDTH * HEIGHT];                                   //canvas target buffer

auto canvas = tvg::SwCanvas::gen();                                       //generate a canvas
canvas->target(buffer, WIDTH, WIDTH, HEIGHT, tvg::ColorSpace::ARGB8888);  //buffer, stride, w, h, Colorspace
```

Next you can draw multiple shapes on the canvas:

```cpp
auto rect = tvg::Shape::gen();               //generate a shape
rect->appendRect(50, 50, 200, 200, 20, 20);  //define it as a rounded rectangle (x, y, w, h, rx, ry)
rect->fill(100, 100, 100);                   //set its color (r, g, b)
canvas->push(rect);                          //push the rectangle into the canvas

auto circle = tvg::Shape::gen();             //generate a shape
circle->appendCircle(400, 400, 100, 100);    //define it as a circle (cx, cy, rx, ry)

auto fill = tvg::RadialGradient::gen();      //generate a radial gradient
fill->radial(400, 400, 150);                 //set the radial gradient geometry info (cx, cy, radius)

tvg::Fill::ColorStop colorStops[2];          //gradient colors
colorStops[0] = {0.0, 255, 255, 255, 255};   //1st color values (offset, r, g, b, a)
colorStops[1] = {1.0, 0, 0, 0, 255};         //2nd color values (offset, r, g, b, a)
fill->colorStops(colorStops, 2);             //set the gradient colors info

circle->fill(fill);                          //set the circle fill
canvas->push(circle);                        //push the circle into the canvas

```

This code generates the following result:

<p align="center">
  <img width="416" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_shapes.png">
</p>

You can also draw you own shapes and use dashed stroking:

```cpp
auto path = tvg::Shape::gen();               //generate a path
path->moveTo(199, 34);                       //set sequential path coordinates
path->lineTo(253, 143);
path->lineTo(374, 160);
path->lineTo(287, 244);
path->lineTo(307, 365);
path->lineTo(199, 309);
path->lineTo(97, 365);
path->lineTo(112, 245);
path->lineTo(26, 161);
path->lineTo(146, 143);
path->close();

path->fill(150, 150, 255);                   //path color

path->strokeWidth(3);                        //stroke width
path->strokeFill(0, 0, 255);                 //stroke color
path->strokeJoin(tvg::StrokeJoin::Round);    //stroke join style
path->strokeCap(tvg::StrokeCap::Round);      //stroke cap style

float pattern[2] = {10, 10};                 //stroke dash pattern (line, gap)
path->strokeDash(pattern, 2);                //set the stroke pattern

canvas->push(path);                          //push the path into the canvas

```

The code generates the following result:

<p align="center">
  <img width="300" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_path.png">
</p>

Now begin rendering & finish it at a particular time:

```cpp
canvas->draw();
canvas->sync();
```

Then you can acquire the rendered image from the buffer memory.

Lastly, terminate the engine after its usage:

```cpp
tvg::Initializer::term();
```
[Back to contents](#contents)
<br />
<br />
## SVG

ThorVG facilitates [SVG Tiny Specification](https://www.w3.org/TR/SVGTiny12/) rendering via its dedicated SVG interpreter. Adhering to the SVG Tiny Specification, the implementation maintains a lightweight profile, rendering it particularly advantageous for embedded systems. While ThorVG comprehensively adheres to [most of the SVG Tiny specs](https://github.com/thorvg/thorvg/wiki/SVG-Support), certain features remain unsupported within the current framework. These include:</br>

 - Animation 
 - Interactivity
 - Multimedia

The figure below highlights ThorVG's SVG rendering capabilities:

<p align="center">
  <img width="780" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_svg.jpg">
</p>

The following code snippet shows how to draw SVG image using ThorVG:

```cpp
auto picture = tvg::Picture::gen();         //generate a picture
picture->load("tiger.svg");                 //load a SVG file
canvas->push(picture);                      //push the picture into the canvas
```

The result is:

<p align="center">
  <img width="300" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_tiger.png">
</p>

[Back to contents](#contents)
<br />
<br />
## Lottie

ThorVG supports the most powerful Lottie Animation features ([see dotLottie Player](https://lottiefiles.com/supported-features)). Lottie is an industry standard, JSON-based vector animation file format that enables seamless distribution of animations on any platform, akin to shipping static assets. These files are compact and compatible with various devices, scaling up or down without pixelation. With Lottie, you can easily create, edit, test, collaborate, and distribute animations in a user-friendly manner. For more information, please visit [Lottie Animation Community](https://lottie.github.io/)' website. <br />
<br />
ThorVG offers great flexibility in building its binary. Besides serving as a general graphics engine, it can be configured as a compact Lottie animation playback library with specific build options:

```
$meson setup builddir -Dloaders="lottie"
```

Alternatively, to support additional bitmap image formats:

```
$meson setup builddir -Dloaders="lottie, png, jpg, webp"
```

Please note that ThorVG supports Lottie Expressions by default. Lottie Expressions are small JavaScript code snippets that can be applied to animated properties in your Lottie animations, evaluating to a single value. This is an advanced feature in the Lottie specification and may impact binary size and performance, especially when targeting small devices such as MCUs. If this feature is not essential for your requirements, you can disable it using the `extra` build option in ThorVG:

```
$meson setup builddir -Dloaders="lottie" -Dextra=""
```

The following code snippet demonstrates how to use ThorVG to play a Lottie animation.

```cpp
auto animation = tvg::Animation::gen();     //generate an animation
auto picture = animation->picture()         //acquire a picture which associated with the animation.
picture->load("lottie.json");               //load a Lottie file
auto duration = animation->duration();      //figure out the animation duration time in seconds.
canvas->push(picture);                      //push the picture into the canvas
```
First, an animation and a picture are generated. The Lottie file (lottie.json) is loaded into the picture, and then the picture is added to the canvas. The animation frames are controlled using the animation object to play the Lottie animation. Also you might want to know the animation duration time to run your animation loop.
```cpp
animation->frame(animation->totalFrame() * progress);  //Set a current animation frame to display
canvas->update(animation->picture());                  //Update the picture to be redrawn.
```
Let's suppose the progress variable determines the position of the animation, ranging from 0 to 1 based on the total duration time of the animation. Adjusting the progress value allows you to control the animation at the desired position. Afterwards, the canvas is updated to redraw the picture with the updated animation frame.<br />
<br />
<p align="center">
  <img width="600" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_lottie.gif">
</p>

Please check out the [ThorVG Test App](https://thorvg-perf-test.vercel.app/) to see the performance of various Lottie animations powered by ThorVG.

[Back to contents](#contents)
<br />
<br />
## In Practice
### Canva iOS
[Canva](https://www.canva.com), a leading visual communication platform, is a household name among creators, marketers, designers, students, and more, with millions of users worldwide. It empowers users to create stunning visual content with a user-friendly interface and a vast library of templates and design elements. The Canva iOS app transitioned from the existing Lottie animation engine to ThorVG for Lottie animations, resulting in approximately an 80% improvement in rendering speed and a 70% reduction in peak memory usage.
<p align="center">
  <img width="700" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_canvaios.png">
</p>

### dotLottie
[dotLottie](https://dotlottie.io/) is an open-source file format that aggregates one or more Lottie files and their associated resources, such as images and fonts, into a single file. This enables an efficient and easy distribution of animations. dotLottie files are ZIP archives compressed with the Deflate compression method and carry the file extension of “.lottie”. Think of it as a superset of Lottie. [LottieFiles](https://lottiefiles.com/) aims to achieve just that. [dotLottie player](https://github.com/LottieFiles/dotlottie-rs) by LottieFiles is now powered by ThorVG.
<p align="center">
  <img width="700" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_dotlottie.png">
</p>

### Flux Audio
[Flux Audio](https://www.flux.audio/) is a cutting-edge audio technology company specializing in high-fidelity sound systems and immersive audio experiences. With a focus on delivering precision and quality, Flux Audio leverages advanced software solutions to enhance audio processing across a wide range of devices. ThorVG is integrated into the user interface of Flux products, providing efficient and scalable vector rendering for their visual elements, ensuring a sleek and responsive user experience. This collaboration highlights ThorVG’s versatility in high-performance audio platforms.

<p align="center">
  <img width="800" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_flux.jpg">
</p>

### Godot
ThorVG has been integrated into the [Godot](https://www.godotengine.org) project to enable the creation of sleek and visually appealing user interfaces (UIs) and vector resources in the Godot game engine. Godot is a modern game engine that is both free and open-source, offering a comprehensive range of tools. With Godot, you can concentrate on developing your game without the need to recreate existing functionalities.

<p align="center">
  <img width="798" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_godot.png">
</p>

### Lottie Creator

[Lottie Creator](https://creator.lottiefiles.com/) is designed to create ultra-lightweight, highly customizable and interactive animations for web, apps and social. Supercharged with AI-based Motion Copilot. ThorVG is powering the Canvas engine behind Lottie Creator — enabling fast and scalable vector graphics rendering across platforms.

<p align="center">
  <img width="800" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_creator.png">
</p>

### LVGL
[LVGL](https://lvgl.io/) is an open-source graphics library specifically designed for embedded systems with limited resources. It is lightweight and highly customizable, providing support for graphical user interfaces (GUIs) on microcontrollers, IoT devices, and other embedded platforms. ThorVG serves as the vector drawing primitives library in the LVGL framework.

<p align="center">
  <img width="700" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_lvgl.png">
</p>

### Tizen
ThorVG has been integrated into the [Tizen](https://www.tizen.org) platform as the vector graphics engine. [NUI](https://docs.tizen.org/application/dotnet/guides/user-interface/nui/overview/) is the name of Tizen UI framework which is written in C#. ThorVG is the backend engine of the [NUI Vector Graphics](https://docs.tizen.org/application/dotnet/guides/user-interface/nui/vectorgraphics/Overview/) which is used for vector primitive drawings and scalable image contents such as SVG and Lottie Animation among the Tizen applications.

<p align="center">
  <img width="798" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_tizen.png">
</p>

[Back to contents](#contents)
<br />
<br />
## References
- [Universal Motion Graphics across All Platforms: Unleashing Creativity with ThorVG](https://youtu.be/qhHMycRPQ9M?si=RXAag3Fxm8R7W_I0)
- [Canva Enhances iOS Lottie Rendering: 80% Faster and 70% More Efficient with ThorVG](https://lottiefiles.com/blog/working-with-lottie-animations/canva-enhances-ios-rendering-faster-and-efficient-with-thorvg)

[Back to contents](#contents)
<br />
<br />
## Documentation
The ThorVG API documentation is available at [thorvg.org/apis](https://www.thorvg.org/apis), and can also be found directly in this repository via the [C++ API](https://github.com/thorvg/thorvg/blob/main/inc/thorvg.h) and [C API](https://github.com/thorvg/thorvg/blob/main/src/bindings/capi/thorvg_capi.h). 

For comprehensive and well-structured technical information, please visit the [DeepWiki](https://deepwiki.com/thorvg/thorvg), which offers in-depth guidance on ThorVG's architecture, features, and usage.

[Back to contents](#contents)
<br />
<br />
## Examples
There are plenty of sample code in `thorvg/examples` to help you in understanding the ThorVG APIs.

To execute these examples, you can build them with the following meson build option:
```
meson setup builddir -Dexamples=true
```
Note that these examples require the SDL dev package for launching. If you're using Linux-based OS, you can easily install this package from your OS distribution server. For Ubuntu, you can install it with this command.
```
apt-get install libsdl2-dev
```
Alternatively, you can read the official guidance [here](https://wiki.libsdl.org/SDL2/Installation) for other platforms. Fore more information, please visit the official [SDL](https://www.libsdl.org/) site.

[Back to contents](#contents)
<br />
<br />
## Tools
### ThorVG Viewer
ThorVG provides the resource verification tool for the ThorVG Engine. [ThorVG viewer](https://thorvg.github.io/thorvg.viewer/) does immediate rendering via web browser running on the ThorVG web-assembly binary, allowing real-time editing of the vector elements on it. It doesn't upload your resources to any external server while allowing to export to supported formats such as GIF, so the designer resource copyright is protected.</br>
</br>

<p align="center">
  <img width="700" height="auto" src="https://github.com/thorvg/thorvg/assets/3711518/edadcc5e-3bbf-489d-a9a1-9570079c7d55"/>
</p>

### Lottie to GIF
ThorVG provides an executable `tvg-lottie2gif` converter that generates a GIF file from a Lottie file.

To use the `tvg-lottie2gif`, you must turn on this feature in the build option:
```
meson setup builddir -Dtools=lottie2gif -Dsavers=gif
```
To use the 'tvg-lottie2gif' converter, you need to provide the 'Lottie files' parameter. This parameter can be a file name with the '.json' extension or a directory name. It also accepts multiple files or directories separated by spaces. If a directory is specified, the converter will search for files with the '.json' extension within that directory and all its subdirectories.<br />
<br />
Optionally, you can specify the image resolution in the 'WxH' format, with two numbers separated by an 'x' sign, following the '-r' flag.<br />
<br />
Both flags, if provided, are applied to all of the `.json` files.

The usage examples of the `tvg-lottie2gif`:
```
Usage:
    tvg-lottie2gif [Lottie file] or [Lottie folder] [-r resolution] [-f fps] [-b background color]

Flags:
    -r set the output image resolution.
    -f specifies the frames per second (fps) for the generated animation.
    -b specifies the base background color (RGB in hex). If not specified, the background color will follow the original content.

Examples:
    $ tvg-lottie2gif input.json
    $ tvg-lottie2gif input.json -f 30
    $ tvg-lottie2gif input.json -r 600x600 -f 30
    $ tvg-lottie2gif lottiefolder
    $ tvg-lottie2gif lottiefolder -r 600x600
    $ tvg-lottie2gif lottiefolder -r 600x600 -f 30 -b fa7410
```

### SVG to PNG
ThorVG provides an executable `tvg-svg2png` converter that generates a PNG file from an SVG file.

To use the `tvg-svg2png`, you must turn on this feature in the build option:
```
meson setup builddir -Dtools=svg2png
```
To use the 'tvg-svg2png' converter, you need to provide the 'SVG files' parameter. This parameter can be a file name with the '.svg' extension or a directory name. It also accepts multiple files or directories separated by spaces. If a directory is specified, the converter will search for files with the '.svg' extension within that directory and all its subdirectories.<br />
<br />
Optionally, you can specify the image resolution in the 'WxH' format, with two numbers separated by an 'x' sign, following the '-r' flag.<br />
<br />
The background color can be set with the `-b` flag. The `bgColor` parameter should be passed as a three-bytes hexadecimal value in the `ffffff` format. The default background is transparent.<br />
<br />
Both flags, if provided, are applied to all of the `.svg` files.

The usage examples of the `tvg-svg2png`:
```
Usage:
    tvg-svg2png [SVG files] [-r resolution] [-b bgColor]

Flags:
    -r set the output image resolution.
    -b set the output image background color.

Examples:
    $ tvg-svg2png input.svg
    $ tvg-svg2png input.svg -r 200x200
    $ tvg-svg2png input.svg -r 200x200 -b ff00ff
    $ tvg-svg2png input1.svg input2.svg -r 200x200 -b ff00ff
    $ tvg-svg2png . -r 200x200
```

[Back to contents](#contents)
<br />
<br />
## API Bindings
Our main development APIs are written in C++, but ThorVG also provides API bindings for C.

To enable CAPI binding, you need to activate this feature in the build options:
```
meson setup builddir -Dbindings="capi"
```
[Back to contents](#contents)
<br />
<br />
## Dependencies
ThorVG provides flexible image loading capabilities, supporting both static and external loaders. This design ensures that even in environments lacking external libraries, users can rely on built-in static loaders for core functionality. At its core, the ThorVG library is fully self-contained and operates without mandatory external dependencies. However, several optional feature extensions are available, each with its own set of dependencies.

The following outlines the dependencies for these optional features:

* **GL Engine**: [OpenGL 3.3](https://www.khronos.org/opengl/), [OpenGL ES 3.0](https://www.khronos.org/opengles/), or a browser with [WebGL2](https://www.khronos.org/webgl/) support.
* **WG Engine**: [webgpu-native v0.22](https://github.com/gfx-rs/wgpu-native) or a browser with [WebGPU](https://www.w3.org/TR/webgpu/) support.
* **PNG Loader** (external): [libpng](https://github.com/pnggroup/libpng)
* **JPEG Loader** (external): [libjpeg-turbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
* **WebP Loader** (external): [libwebp](https://developers.google.com/speed/webp/download)
* **Examples**: [SDL2](https://www.libsdl.org/)

[Back to contents](#contents)
<br />
<br />
## Contributors
ThorVG stands as a purely open-source initiative. We are grateful to the individuals, organizations, and companies that have contributed to the development of the ThorVG project. The dedicated efforts of the individuals and entities listed below have enabled ThorVG to reach its current state.

* [Individuals](https://github.com/thorvg/thorvg/blob/main/CONTRIBUTORS)
* [Canva Pty Ltd](https://www.canva.com/)
* [LottieFiles](https://lottiefiles.com/) by Design Barn Inc.
* Samsung Electronics Co., Ltd

[Back to contents](#contents)
<br />
<br />
## Sponsors
We extend our gratitude to our financial sponsors, whose generous support empowers the ThorVG project. Their funding enables us to continually enhance and expand this open-source project, making it more powerful, efficient, and accessible for the entire community.
<br/>
<br/>
<p align="center", href="https://lottiefiles.com">
  <a href="https://lottiefiles.com">
  <img width="250" height="auto" src="https://github.com/thorvg/thorvg/blob/main/res/example_lottiefiles.jpg"  alt="LottieFiles">
  </a>
</p>
<br/>
We are also seeking your support to ensure the continued development of the ThorVG project. Your generous donations will help cover operational costs and contribute to the growth of this open-source project. Even a small contribution can make a big difference in securing the future of ThorVG!
<br/>
<br/>

* [Open Collective](https://opencollective.com/thorvg)

[Back to contents](#contents)
<br />
<br />
## Communication
For real-time conversations and discussions, please join us on [Discord](https://discord.gg/n25xj6J6HM)

[Back to contents](#contents)
