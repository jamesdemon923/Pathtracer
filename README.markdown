# Implement path tracing

## Set up

**Operating & compiling environment**:

* Ubuntu (WSL2 in Windows11)

**Software rendering**:

* CPU: AMD 5800H

**Model**:

* Cornell Box

**Command**:

* cmake ..
* make

## Build the BVH

<div align=center>
    <img src="image\principle\BVH1.png" width="300"/><img src="image\principle\BVH2.png" width="350"/>
</div>

Pseudocode:

```
function BVHAccel(objects)
    Create a new node
    Calculate the bounding box of all objects
    Assign it to the node
    If there is only one object
        Create a leaf node
        Assign object to this node
        Return this node
    Else
        Calculate the centroid of the bounding box
        Determine the dimension (x, y, or z) with the maximum spread of centroids
        Sort objects based on their centroid position along this dimension
        Divide the objects into two groups
        Recursively call BVHAccel on both groups to create left and right children
        Calculate the bounding box of node as union of bounding boxes of children
        Return this node
    End If
End function
```

Use the **Surface Area Heuristic (SAH)** to solve the problem that objects are not uniformly distributed.


$$
C = C_{trav} + \frac {S_{A}}{S_{N}} N_{A} C_{isect}+ \frac {S_{B}}{S_{N}} N_{B} C_{isect}
$$

```
For each axis (x, y, z)
    Sort objects based on their centroid position along this axis
    For each possible split position along sorted objects
        Calculate the bounding boxes and counts for objects on the left and right of split
        Calculate cost of this configuration using SAH
    End For
End For
```

## Patch tracing

### Beginning

**Rendering equation**:

<div align=center>
    <img src="image\principle\Rendering equation.jpg" alt="Rendering equation" width="350" />
</div>   

**Monte Carlo integration**:



$$
\int_{a}^{b}f(x)dx=\frac{1}{N}\sum_{i=1}^{N}\frac{f(X_{i})}{p(X_{i})}
$$


**Global illumination** = Direct illumination + indirect illumination



$$
L = E + KE + K^{2}E + K^{3}E + ...
$$

### Russian roulette

Previously, we always shoot a ray at a shading point and get the shading result $\mathbf{L_{o}}$. Suppose we manually set a probability P (0 < P < 1) With probability P, shoot a ray and return the shading result divided by P: $\mathbf{L_{o}}$ / P With probability 1-P, don’t shoot a ray and you’ll get 0. In this way, you can still expect to get $\mathbf{L_{o}}$:


$$
E = P * (\mathbf{L_{o}} / P) + (1 - P) * 0 = \mathbf{L_{o}}
$$

### Sample light source

<div align=center>
    <img src="image\principle\Sample the light.jpg" alt="Sample the light" width="350" />
</div>    

The rendering equation becomes:



$$
\begin{align}
L_{o}(x,\omega_{o}) & = \int_{\Omega^{+}} L_{i}(x,\omega_{i})f_{r}(x,\omega_{i},\omega_{o})\cos\theta d\omega_{i}\\
& = \int_{A} L_{i}(x,\omega_{i})f_{r}(x,\omega_{i},\omega_{o})\frac{\cos\theta \cos\theta^{\prime}}{\left \|x^{\prime} - x \right \|^{2} } dA 
\end{align}
$$

### Pseudocode of Algorithm:

```c++
shade (p, wo)
    sampleLight ( inter , pdf_light )
    Get x, ws , NN , emit from inter
    Shoot a ray from p to x
    If the ray is not blocked in the middle
        L_dir = emit * eval(wo , ws , N) * dot(ws , N) * dot(ws , NN) / |x-p|^2 / pdf_light
    
    L_indir = 0.0
    Test Russian Roulette with probability RussianRoulette
    wi = sample (wo , N)
    Trace a ray r(p, wi)
    If ray r hit a non - emitting object at q
        L_indir = shade (q, -wi) * eval (wo , wi , N) * dot(wi , N) / pdf(wo , wi , N) / RussianRoulette
    
    Return L_dir + L_indir
```

## Enhancement

### Multi-threaded acceleration

Maximize the amount of computation per thread, and use multiple threads for the entire frame computation.

In this project,the rendering process can be accelerated by using multiple threads to chunk the image and perform path tracing for each part synchronously. Here, the rendering (process) implemented by Render() (program) can be executed concurrently using fragments (thread) for multiple pixel operations (multiple threads).

```c++
#include <mutex>
#include <thread>
    ...
    int thread_num = 20; // AMD5800H, based on your own cpu
    int thread_height = scene.height / thread_num;
    // thread is a right-valued reference
    std::vector<std::thread> threads(thread_num);
    ...
    std::mutex mtx; // Solve two multi-threaded resource contention problems
    float process = 0;
    float Reciprocal_Scene_height=1.f/ (float)scene.height;
    // Refer to lambda function
    auto castRay = [&](int thread_index) 
    {
        int height = thread_height * (thread_index + 1);
        for (uint32_t j = height - thread_height; j < height; j++)
        {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++) {
                    framebuffer[j*scene.width+i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                }
            }
            mtx.lock();
            process = process + Reciprocal_Scene_height;
            UpdateProgress(process); // Every thread is used to update the process
            mtx.unlock();
        }
    };
    // from the 0th thread
    for (int k = 0; k < thread_num; k++)
    {
        threads[k] = std::thread(castRay,k);
    }
    // Execute join once for each thread
    for (int k = 0; k < thread_num; k++)
    {
        threads[k].join();
    }

    UpdateProgress(1.f);
```

### Microfacet material

Microfacet BRDF:

<div align=center>
    <img src="image\principle\Microfacet BRDF.jpg" alt="Microfacet BRDF" width="350" />
</div>    

We use the GGX for $G(\mathbf{i,o,h})$ and $D(\mathbf{h})$

<div align=center>
    <img src="image\principle\GGX NDF.png" alt="GGX NDF" width="450" />
</div>

```c++
// Define the microfacet material in main.cpp
Material* microfacet = new Material(MICROFACET, Vector3f(0.0f));
microfacet->Ks = Vector3f(0.45, 0.45, 0.45);
microfacet->Kd = Vector3f(0.3, 0.3, 0.25);
microfacet->ior = 12.85;

// In sample and pdf function, it is the same as DIFFUSE

// In eval function
case MICROFACET:
        {
            // Implement the Torrance-Sparrow Model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) 
            {
                // calculate the contribution of Microfacet model
                float F, G, D;

                fresnel(wi, N, ior, F);

                float Roughness = 1;// roughness
                auto G_function = [&](const float& Roughness, const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
                {
                    float A_wi, A_wo;
                    A_wi = (-1 + sqrt(1 + Roughness * Roughness * pow(tan(acos(dotProduct(wi, N))), 2))) / 2;
                    A_wo = (-1 + sqrt(1 + Roughness * Roughness * pow(tan(acos(dotProduct(wo, N))), 2))) / 2;
                    float divisor = (1 + A_wi + A_wo);
                    if (divisor < 0.001)
                        return 1.f;
                    else
                        return 1.0f / divisor;
                };
                G = G_function(Roughness, -wi, wo, N);

                auto D_function = [&](const float& Roughness, const Vector3f& h, const Vector3f& N)
                {
                    float cos_sita = dotProduct(h, N);
                    float divisor = (M_PI * pow(1.0 + cos_sita * cos_sita * (Roughness * Roughness - 1), 2));
                    if (divisor < 0.001)
                        return 1.f;
                    else 
                        return (Roughness * Roughness) / divisor;
                };
                Vector3f h = normalize(-wi + wo);
                D = D_function(Roughness, h, N);

                // energy balance
                Vector3f diffuse = (Vector3f(1.0f) - F) * Kd / M_PI;
                Vector3f specular;
                float divisor= ((4 * (dotProduct(N, -wi)) * (dotProduct(N, wo))));
                if (divisor < 0.001)
                    specular= Vector3f(1);
                else
                    specular = F *G * D / divisor;
                //std::cout << "F:"<<F << "\n";
                //std::cout << "diffuse:"<<diffuse<<"\n";
                //std::cout << "specular:" << specular << "\n";
                return diffuse+specular;
            }
            else
                return Vector3f(0.0f);
            break;
        }
```

Remove the black noise:

```c++
In Intersection getIntersection(Ray ray):

if (t0 > 0.5) // Remove black noise
{
    result.happened = true;
    result.coords = Vector3f(ray.origin + ray.direction * t0);
    result.normal = normalize(Vector3f(result.coords - center));
    result.m = this->m;
    result.obj = this;
    result.distance = t0;
}
```

Remove the white noise:

```c++
In Vector3f Scene::castRay(const Ray &ray, int depth) const:

(hitColor).x= (clamp(0, 1, (hitColor).x));
(hitColor).y = (clamp(0, 1, (hitColor).y));
(hitColor).z = (clamp(0, 1, (hitColor).z));
```

### Perfect mirror reflection model

In this model, only the coefficient of Fresnel reflection needs to be considered.

The BRDF of Perfect mirror reflection model:


$$
f_{r}(p,\omega_{o},\omega_{i})=F_{r}(\omega_{r})\frac{\delta (\omega_{i} - \omega_{r})}{\cos \theta_{i}}
$$


Implementation:

```c++
Material* mirror = new Material(MIRROR, Vector3f(0.0f));
mirror->Ks = Vector3f(0.45, 0.45, 0.45);
mirror->Kd = Vector3f(0.3, 0.3, 0.25);
mirror->ior = 12.85;

// in sample
case MIRROR:
{
    Vector3f localRay = reflect(wi, N);
    return localRay;
    break;
}
// in pdf
case MIRROR:
{
    if (dotProduct(wo, N) > 0.0f)
        return 1.0f;
    else
        return 0.0f;
    break;
}
// in eval
case MIRROR:
{
    float cosalpha = dotProduct(N, wo);
    if (cosalpha > 0.0f) 
    {
        float divisor = cosalpha;
        if (divisor < 0.001) return 0;
        Vector3f mirror = 1 / divisor;
        float F;
        fresnel(wi, N, ior, F);
        return F * mirror;
    }
    else
        return Vector3f(0.0f);
    break;
}

// also need to do importance sampling by modifying the castRay to prevent overexposure (only sample the indir light)
case MIRROR:
{
    float ksi = get_random_float(); // random number

    if (ksi < RussianRoulette){
        Vector3f wi = inter.m->sample(wo, N).normalized();

        Ray r(p, wi);
        Intersection obj_to_scene = Scene::intersect(r);

        // hit obj instead of light source
        if (obj_to_scene.happened) {
            Vector3f f_r = inter.m->eval(wo, wi, N);
            float cos_theta = dotProduct(wi, N);
            float pdf_hemi = inter.m->pdf(wo, wi, N);
            indir = castRay(r, depth + 1) * f_r * cos_theta / pdf_hemi / RussianRoulette;
        }
    }
	break;
}
```

## Result

<table>
     <tr>
        <th colspan="5">Different SPP (sample per pixel):</th>
    </tr>
    <tr>
        <td ><center><img src="image/result/spp2.jpg" >SPP = 2 </center></td>
        <td ><center><img src="image/result/spp4.jpg"  >SPP = 4</center></td>
        <td ><center><img src="image/result/spp16.jpg"  >SPP = 16</center></td>
        <td ><center><img src="image/result/spp64.jpg"  >SPP = 64</center></td>
        <td ><center><img src="image/result/spp128.jpg"  >SPP = 128</center></td>
    </tr>

<table>
    <tr>
        <th colspan="1">Accelerated</th>
        <th colspan="1">Non-accelerated</th>
    </tr>
    <tr>
        <td ><center><img src="image/result/spp2_acc_time.jpg" >SPP=2 with acc </center></td>
        <td ><center><img src="image/result/spp2_noacc_time.jpg"  >SPP=2 without acc</center></td>
    </tr>
    <tr>
        <td ><center><img src="image/result/spp4_acc_time.jpg" >SPP=4 with acc </center></td>
        <td ><center><img src="image/result/spp4_noacc_time.jpg"  >SPP=4 without acc</center></td>
    </tr>
    <tr>
        <td ><center><img src="image/result/spp16_acc_time.jpg" >SPP=16 with acc </center></td>
        <td ><center><img src="image/result/spp16_noacc_time.jpg"  >SPP=16 without acc</center></td>
    </tr>
        <tr>
        <td ><center><img src="image/result/spp64_acc_time.jpg" >SPP=64 with acc </center></td>
        <td ><center><img src="image/result/spp64_noacc_time.jpg"  >SPP=64 without acc</center></td>
    </tr>
        <tr>
        <td ><center><img src="image/result/spp128_acc_time.jpg" >SPP=128 with acc </center></td>
        <td ><center><img src="image/result/spp128_noacc_time.jpg"  >SPP=128 without acc</center></td>
    </tr>

<table>
    <tr>
        <th colspan="4">Using microfacet (SPP=64):</th>
    </tr>
    <tr>
        <td ><center><img src="image/result/spp64_rough0.jpg" >roughness=0</center></td>
        <td ><center><img src="image/result/spp64_rough0.25.jpg"  >roughness=0.25</center></td>
        <td ><center><img src="image/result/spp64_rough0.5.jpg"  >roughness=0.5</center></td>
        <td ><center><img src="image/result/spp64_rough1.jpg"  >roughness=1</center></td>
    </tr>
<table>
    <tr>
        <th colspan="1">Perfect mirror reflection model:</th>
    </tr>
    <tr>
        <td ><center><img src="image/result/Perfect mirror reflection model spp128 rough0.25.jpg" >SPP=128, roughness=0.25</center></td>
    </tr>
