![image](https://github.com/chrylt/VPCR/assets/33519687/ab812c31-c774-474b-b594-ab8c79b4875c)


# Vulkan Point Cloud Renderer (VPCR)

Vulkan Point Cloud Renderer (VPCR) is a high-performance, real-time point cloud rendering software that builds on the Vulkan-based framework provided by [TGA](https://github.com/Estard/TGA). It efficiently renders large point clouds, making it ideal for visualization and analysis in various fields such as robotics, computer vision, and geospatial mapping.

## Inspiration and References

The development of VPCR was inspired by and references the following papers:

- Schütz, M., Kerbl, B., & Wimmer, M. (2021). Rendering Point Clouds with Compute Shaders and Vertex Order Optimization.
- Schütz, M., Kerbl, B., & Wimmer, M. (2022). Software Rasterization of 2 Billion Points in Real Time.

In addition to these, we have incorporated our own innovations into the project. We've added Level of Detail (LOD) rendering and different anti-aliasing approaches to enhance the rendering quality. The user-friendly GUI is implemented using the ImGui library.

## Launching the Renderer

To launch the renderer, you need to provide a path to the point cloud file you want to render. VPCR can take `.gltf` or `.glb` files as input. Here's how you can do it:

```bash
# From the build directory, run the following command
./VPCR --scene_file /path/to/your/pointcloud.gltf --resolution 1000 1000
# or
./VPCR --scene_file /path/to/your/pointcloud.glb --resolution 1000 1000
```

Replace `/path/to/your/pointcloud.gltf` or `/path/to/your/pointcloud.glb` with the actual path to the point cloud file you want to render.

## Test Data

We have tested VPCR with several free point clouds available online. We recommend the following for initial testing:

- [Recovering Oak Point Cloud Version](https://sketchfab.com/3d-models/recovering-oak-point-cloud-version-f6dd3f2b081b483bb3f40b6b931016fd)
- [SY Carola Point Cloud](https://sketchfab.com/3d-models/sy-carola-point-cloud-17bd8188447b48baab75125b9ad20788)

Please note that you need to download these point clouds in `.gltf` or `.glb` format before rendering.

## License

This project is licensed under the GNU Affero General Public License. See the [LICENSE](LICENSE) file for details.
