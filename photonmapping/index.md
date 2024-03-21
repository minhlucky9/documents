# I. État de l'art

## 1.1. Modèles d'éclairement
La couleur est une phénomène qui dépend, d'une part, de la physique de la lumière et son interaction avec les matériaux, d'autre part, de l'interprétation des phénomènes résultats par le système visuel humain.

### 1.1.1. La système visuelle d'humain

#### 1.1.1.1. L'oeil humain

<p style="text-align: center">
  <img src="images/eye.png" width=400  style="display:block; margin: auto">
  <br>
  Figure 1: Coupe de l'oeil humain
</p>

La figure 1 présente une coupe de l'oeil humain. L'iris joue le rôle du diaphragme de l'appareil photographique et la pupille celui de la lentile. L'image se forme sur la rétine qui est la surface de l'oeil sensible à la lumière. Sur la rétine, il existe deux types de cellules photo céceptrices qui transforment la lumière en impulsions nerveuses: les cônes et les bâtonnets. Les bâtonnets sont plus sensibles à la lumière que les cônes et permettent la vision nocturne. Les cônes servent à la vision des couleurs et à la vision diurne.


Il existe trois type de cônes, qui diffèrent suivant leur sensibilité aux longueurs d'ondes: certains sont sensibles aux courtes longueurs d'ondes (cônes bleus), les autres ont une sensibilité maximale à 535 nm (cônes verts) ou à 575 nm (cônes rouges).

#### 1.1.1.2. Le mécanisme visuel

Le mécanisme visuel correspond au schéma suivant: un stimulus externe provoque l'activation d'un cône ou d'un bâtonnet qui produit une transition photochimique, laquelle induit de l'influx nerveux. Les impulsions nerveuses fournis par les cônes sont recombinées en trois nouveaux canaux: le premier indique la luminance, le deuxième la différence entre le rouge et le vert et le troisième la différence entre le jaune et le bleu.

### 1.1.2. Modèles locaux

Depuis des années, la synthèse d'images s'applique à définir des modèles de réflexion simulant le plus fidèlement possible la réponse d'un matériau quelconque à une incidence lumineuse. Cependant, l'interaction entre la lumière et la matière est un problème très difficile à modéliser parfaitement en raison de sa complexité. Maintenant, cette interaction est appliquée à de nombreux objectifs tels que la simulation physique ou le rendu temps réel. Et un des techniques qu'on utilise souvent pour ce calcul est la fonction de BRDF.

#### 1.1.2.1. Définition de la BRDF

<p style="text-align: center">
  <img src="images/BRDF.png" width=400  style="display:block; margin: auto">
  <br>
  Figure 2: Le modèle de BRDF
</p>

La luminance est une mesure radiométrique définissant la qualité d'énergie, dans notre cas l'énergie lumineuse, qui est émise ou reçue par une surface élémentaire dans un angle solide élémentaire autour d'une direction donnée. La luminance s'exprime en Watts par unité d'aire et par unité d'angle solide $W.m^{-2}.sr^{-1}$. L'énergie qui arrive sur une portion de surface dans une portion d'angle solide *$d\omega_i$*:

$$dL_i(x, \omega_i) = L_i(x, \omega_i) cos(\vec{N_x}, \omega_i) d\omega_i$$ 

avec:

+ $L_i(x, \omega_i)$ est l'énergie reçcue en x
+ $\omega_i$ est la direction d'éclairement
+ $\vec{N_x}$ est la normale de surface
+ $d\omega_i$ est l'angle solide


La fonction de distribution de la réflectance bidirectionnelle (BRDF) décrit la réflextion d'une onde lumineuse sur une surface. En effet, pour une direction d'éclairement *$\omega_i$* et une direction de réflexion *$\omega_r$*, la BRDF est le rapport de la luminance réflechie en un point *x* d'une surface infinitésimale d'aire *dA* à l'éclairement incident à celle-ci.

$$f_r(x, \omega_i, \omega_r, \lambda) = f_r(x, \theta_i, \phi_i, \theta_r, \phi_r, \lambda) = \frac{dL_r(x,\theta_r, \phi_r, \lambda)}{dL_i(x, \theta_i, \phi_i, \lambda)} = \frac{dL_r(x,\theta_r, \phi_r, \lambda)}{L_i(x, \theta_i, \phi_i, \lambda) cos\theta_i d\omega_i}$$

avec:

+ $\theta_i$ est l'angle entre $\omega_i$ et $\vec{N_x}$
+ $\theta_r$ est l'angle entre $\omega_r$ et $\vec{N_x}$

#### 1.1.2.2. Le modèle de Lambert

<p style="text-align: center">
  <img src="images/Lambert6.gif" width=400 style="display:block; margin: auto;">
  <br>
  Figure 3: Lambertian reflectance
</p>

Le modèle de Lambert, qui suppose une surface parfaitement diffuse. C'est-à-dire que la lumière est réflechie de façon équiprobable par le matériau dans toutes les directions. La BRDF est donc constante et indépendante des directions d'éclairement, de réflexion, et de la longueur d'onde:

$$f_r(\omega_i,\omega_r) = \frac{1}{\pi}$$

En réalité, les surfaces ne réflechissent qu'une partie de la lumière (l'autre étant absorbée). C'est pourquoi on utilise parfois le modèle suivantpour caractériser une surface dite Lambertienne:

$$f_r(\omega_i,\omega_r) = \frac{C}{\pi}$$

avec:\
$C$ est le longueur d'onde de la lumière 

#### 1.1.2.3. Les autres fonctions de BxDF

À côté de la fonction BRDF, il existe aussi des autres fonctions qui s'adaptent aux différents type de matériaux, tels ques:

+ *Bidirectional Transmittance Distribution Function (BTDF):* Le cas où la lumière est réfracté complètement

+ *Bidirectional Scattering Distribution Function (BSDF):* Le cas où une partie de la lumière est diffusée tandis que l'autre est réfracté.

+ *Bidirectional Sous-Surfaque Reflectance Distribution Function (BSSRDF):* Le cas où la lumière est diffusée sous la surface d'un matériel (la peau).

### 1.1.3. Modèles globaux

Dans la section avant, on a vu le modèle qui permet de calculer l'éclairement d'une surface de manière locale, c'est-à-dire sans prendre en compte la participation de l'ensemble des objets constituant une scène dans l'apparence d'un seul objet. En effet, elle ne prennent en compte que la réflexion directe des sources de lumières, alors que la lumière peut subir plusieurs réflexions avant d'atteindre un objet. 

*L'équation de rendu*

L'expression de la lumière $L(x \rightarrow \omega)$ émise en une point *x* d'une surface et dans une direction $\omega$ est répresenté par:

$$ L (x \rightarrow \omega) = L_e(x \rightarrow \omega) + L_r(x \rightarrow \omega)  $$

avec:
* $L_e(x \rightarrow \omega)$ est la luminance propre émise (comme pour une source de lumière)
* $L_r(x \rightarrow \omega)$ est la réflexion de toute la lumière qui arrive sur cette surface

On sait que la réflexion est contrôlée par la fonction BRDF: 

$$ f_r (x, \omega \rightarrow \omega') = \frac{L_r(x \rightarrow \omega)}{L(x \leftarrow \omega')|cos(\vec{N_x}, \omega')|d\omega'}  $$

avec:
* $\omega$ est la direction de réflexion
* $\omega'$ est la direction d'éclarement
* $\vec{N_x}$ est la normale de surface
* $L_r(x \rightarrow \omega)$ est la réflexion de toute la lumière qui arrive sur cette surface
* $L(x \leftarrow \omega')$ est la luminance reçue dans la direction $\omega$'

On a l'équation de rendu au final:

$$ L (x \rightarrow \omega) = L_e(x \rightarrow \omega) + \int_{\Omega} f_r (x, \omega \rightarrow \omega')L(x \leftarrow \omega')|cos(\vec{N_x}, \omega')|d\omega'   $$


## 1.2. Photon Mapping

Le Photon Mapping est la méthode utilisé beaucoup dans les simulations d'interception de la lumière. Il y a deux phases principales dans cette technique, ce sont:
* *Photon Tracing:* Construire la carte de photon
* *Photon Collecting:* Estimer l'énergie lumineuse de chaque pixel d'image

### 1.2.1. Photon Tracing

<p style="text-align: center">
  <img src="images/photon_tracing.png" width=400 style="display:block; margin: auto;">
  <br>
  Figure 4: La phase de Photon Tracing
</p>

Afin de cosntruire la carte de photon, on va lancer plusieurs rayons à partir des sources de lumières et stocker des impacts de chaque rélfexions, ainsi que leurs énergie $\phi$ en Watts (W). Ces impacts sont les *photons* dans notre carte de photon. Si l'émission de N échantillons est guidée par la densité de probabilité $p(x, \omega)$, l'énergie de chaque photon est calculer par cette équation:

$$ \phi = \frac{L(x \rightarrow \omega) |cos(\vec{N_x}, \omega)|}{Np(x,\omega)} $$

avec:
* $N$ est le nombre d'échantions
* $L(x \rightarrow \omega)$ est l'énergie reçue en *x*
* $\vec{N_x}$ est la normale du surface 
* $\omega$ est la direction d'éclairement
* $p(x, \omega)$ est la densité de probabilité (PDF)

Ci-dessous, ce sont des valeurs de PDF qui corresponds quelques cas d'échantillonage:

* Échantillonnage d'une direction uniforme sur une sphère-unité centrée : $p(\omega) = 1/(4\pi)$
* Échantillonnage d'une direction uniforme sur une demi sphère-unité centrée : $p(\omega) = 1/(2\pi)$
* Échantillonnage d'une direction selon le cosinus à la normale : $p(\omega) = |cos(\vec{N}, \omega)|/\pi$
* Échantillonnage d'une direction uniforme d'un triangle d'aire A : $p(\omega) = 1/A$


### 1.2.2. Photon Collecting

<p style="text-align: center">
  <img src="images/photon_colect.gif" width=400 style="display:block; margin: auto;">
  <br>
  Figure 5: La phase de Photon Collecting
</p>

Après d'obtenir la carte de photon, on va l'utiliser pour faire le rendu d'image. Pour chaque pixel d'image, on va lancer un rayon à partir de caméra, puis calculer le point d'intersection *x* entre ce rayon et tous les objets dans la scène. Si S est la surface qui contiens *x* et $p_i$ est la position du $i^{me}$ photon d'énergie $\phi_i$, l'énergie reçue en Watts est: 

$$E = \sum_{p_i \in S }\phi_i$$

ou, en Watts par mètre carré, avec A est l'aire de la surface S:

$$ I = \frac{1}{A}\sum_{p_i \in S }\phi_i$$

# II. Implémentation 

Tous les codes d'implémentation est trouvé sur le Github de Aurélien Besnier dans la lien suivant : https://github.com/AurelienBesnier/photon_mapping 

## 2.1. Matériel

Avant d'implémenter des différents type de matériels, il faut d'implémenter la classe BxDF qui contiens des méthodes de calculer la réflexion de la lumière. Ensuite, toutes les classes de matériels vont héritées cette classe. Maintenant, dans notre projet, on a déjà défini 3 types de BxDF, ce sont:
* **Diffuse :** La réflexion diffuse est parait sur les surface non polies où la lumière est réfléchie dans plusieurs directions. Il est impossible d'avoir un image claire en observant un objet sur ce type de surface
* **Specular :** La réflexion spéculaire est une réflexion régulière de la lumière. Contrairement à la réflexion diffuse, elle ne peut exister que si les rayons lumineux rencontrent une surface parfaitement plane ou polie tels ques les miroirs ou une surface d'eau parfaitement calme
* **Captor :** Ce type de BxDF n'influence pas la calculation de lumière dans la simulation. Il est utilisé seulement pour capturer l'énergie lumineuse dans une région déterminé.   

```cpp
enum class BxDFType {
    DIFFUSE, ///< Diffuse surface
    SPECULAR, ///< Specular surface
    CAPTOR ///< Captor surface
};
```
Ci-dessous, c'est l'implémentation raccourcie de la classe BxDF. Il contient un constructor, une fonction de calculer le BxDF et une fonction d'échantillonner la direction de réflexion. La version complète de ce code est trouvé ici: https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/material.hpp#L49

```cpp
class BxDF {
private:
    BxDFType type; ///< The type of BxDF.

public:
    explicit BxDF(const BxDFType &type) : type(type) {}

    // evaluate BxDF
    virtual Vec3f evaluate(Vec3f &wo, Vec3f &wi,
                           TransportDirection &transport_dir) const = 0;

    // sample direction by BxDF.
    // its pdf is proportional to the shape of BxDF
    virtual Vec3f sampleDirection(Vec3f &wo,
                                  TransportDirection &transport_dir,
                                  Sampler &sampler, Vec3f &wi,
                                  float &pdf) const = 0;
};
```
Dans notre projet, on utilise 4 types de matériels pour construire l'environnement de cette simulation:
* Lambert
* Transparent
* Feuil
* Captor

### 2.1.1. Matériel de Lambert

Le matériel de Lambert est un type de matériel très basique dans notre projet. Il est utilisé pour répresetner la plupart des objets dans la scène tels ques les murs ou la table. 

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/material.hpp#L123

```cpp
class Lambert : public BxDF {
private:
    Vec3f rho;

public:
    explicit Lambert(const Vec3f &rho) : BxDF(BxDFType::DIFFUSE), rho(rho) {}

    Vec3f evaluate(Vec3f &wo, Vec3f &wi,
                   TransportDirection &transport_dir) const override {
        // when wo, wi is under the surface, return 0
        const float cosThetaO = cosTheta(wo);
        const float cosThetaI = cosTheta(wi);
        if (cosThetaO < 0 || cosThetaI < 0) return {0};

        return rho / PI;
    }

    Vec3f sampleDirection(Vec3f &wo,
                          TransportDirection &transport_dir,
                          Sampler &sampler, Vec3f &wi,
                          float &pdf) const override {
        // cosine weighted hemisphere sampling
        wi = sampleCosineHemisphere(sampler.getNext2D(), pdf);

        return evaluate(wo, wi, transport_dir);
    }
};
```

La densité de probabilité (pdf) dans *la partie 1.2.1* est calculer par ce code:

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/sampler.hpp#L103

```cpp
inline Vec3f sampleCosineHemisphere(Vec2f uv, float &pdf) {
    float theta = 0.5f * std::acos(boost::algorithm::clamp(1.0f - 2.0f * uv[0],
                                                           -1.0f, 1.0f));
    float phi = PI_MUL_2 * uv[1];
    float cosTheta = std::cos(theta);
    pdf = PI_INV * cosTheta;
    Vec3f cart = sphericalToCartesian(theta, phi);
    return cart;
}
```

### 2.1.2. Matériel de transparence

Comme le matériel de Lambert, le matériel de transparence est aussi un matériel très basique. Il est utilisé pour répresenter les matériaux qui permettent la lumière de traverser tels ques les verres, l'eau, etc.

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/material.hpp#L404

```cpp
class Transparent : public BxDF {
private:
    Vec3f rho;
    float ior;

public:
    Transparent(const Vec3f &rho, float ior)
            : BxDF(BxDFType::DIFFUSE), rho(rho), ior(ior) {}

    // NOTE: delta function
    Vec3f evaluate(Vec3f &wo, Vec3f &wi,
                   TransportDirection &transport_dir) const override {
        const float cosThetaO = cosTheta(wo);
        const float cosThetaI = cosTheta(wi);
        if (cosThetaO < 0 || cosThetaI < 0) return {0};

        return rho / PI;
    }

    Vec3f sampleDirection(Vec3f &wo,
                          TransportDirection &transport_dir,
                          Sampler &sampler, Vec3f &wi,
                          float &pdf) const override {
        // set appropriate ior, normal
        float iorO, iorI;
        Vec3f n;
        if (wo[1] > 0) {
            iorO = 1.0f;
            iorI = ior;
            n = Vec3f(0, 1, 0);
        } else {
            iorO = ior;
            iorI = 1.0f;
            n = Vec3f(0, -1, 0);
        }

        // fresnel reflectance
        const float fr = fresnelR(dot(wo, n), iorO, iorI);

        // reflection
        if (sampler.getNext1D() < fr) {
            wi = reflect(wo, n);
            pdf = 1.0f;

            //bxdf = 1/(PI * pdf) = 1 / cosTheta
            return rho / absCosTheta(wi);
        }
            // refraction
        else {
            Vec3f tr;
            if (refract(wo, n, iorO, iorI, tr)) {
                wi = tr;
                pdf = 1.0f;

                float scaling = 1.0f;
                if (transport_dir == TransportDirection::FROM_CAMERA) {
                    scaling = (iorO * iorO) / (iorI * iorI);
                }

                return scaling * rho / absCosTheta(wi);
            }
                // total reflection
            else {
                wi = reflect(wo, n);
                pdf = 1.0f;
                return rho / absCosTheta(wi);
            }
        }
    }

};
```

### 2.1.3. Matériel de feuil et de capteur

L'implémentation de ces deux matériaux est pareil que celle de matériel de transparence, sauf que l'ior (index of reflectance) de feuil est 1.425 et l'ior de capteur est 1. 

L'article concernant l'ior de feuil: https://opg.optica.org/ao/abstract.cfm?uri=ao-13-1-109

## 2.2. Lumière

<p style="text-align: center">
  <img src="images/lumière.png" style="display:block; margin: auto;">
  <br>
  Figure 6: Des types de source de la lumière
</p>

Dans un moteur de rendu, il existe plusieurs types de sources de la lumière qui va nous donner des résultats différents de rendu en appliquant dans la scène. Afin de simuler ces types de lumière, on va échantillonner des rayons de lumière qui partent de ces sources. Chaque rayon de lumière peut être répresenté par ces facteurs:

* Le point de départ
* La direction de la lumière
* L'énergie lumineuse de ce rayon

Ci-dessous, c'est l'implémentation de lumière en général. Il va contenir une variable pour stocker l'énergie totale de cette source, des fonctions pour générer le point de départ ainsi que la direction de rayon.  

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/light.hpp#L23C1-L37C3

```cpp
class Light {
public:
    virtual ~Light() = default;

    /**
    * @fn Vec3f Le() override
    * Get the light emission of the light source.
    * @returns the le attribute.
    */
    virtual Vec3f Le() = 0;

    virtual SurfaceInfo samplePoint(Sampler &sampler, float &pdf) = 0;

    virtual Vec3f sampleDirection(const SurfaceInfo &surfInfo, Sampler &sampler, float &pdf) = 0;
};
```

Dans cette simulation, on a déjà implémenté 4 types de lumières correspondant à 4 stratégies d'échantillonage de lumière, ce sont:

* Point Light
* Spot Light
* Tube Light
* Area Light

### 2.2.1. Point light

### 2.2.2. Spot light

### 2.2.3. Tube light

### 2.2.4. Area light

## 2.3. Scène

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/scene.hpp#L151C1-L166C80

```cpp
  std::vector<float> vertices;   ///< The vertices of the scene.
  std::vector<uint32_t> indices; ///< The indices of the scene.
  std::vector<float> normals;    ///< The normals of the scene.

  std::vector<boost::optional<tinyobj::material_t>>
      materials; ///< The materials of the scene.

  std::vector<Triangle> triangles; ///< The triangles of the scene per face.

  std::vector<boost::shared_ptr<BxDF>>
      bxdfs; ///< The bxdfs of the scene per face.

  std::vector<boost::shared_ptr<Light>>
      lights; ///< The lights of the scene per face.

  std::vector<Primitive> primitives; ///< The primitives of the scene per face.
```

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/scene.hpp#L380C3-L426C4

```cpp
  void addLight(std::vector<float> newVertices,
                std::vector<uint32_t> newIndices, std::vector<float> newNormals,
                float intensity, Vec3f color) {
    for (uint32_t &i : newIndices) {
      i += nVertices();
    }
    this->vertices.insert(std::end(this->vertices), std::begin(newVertices),
                          std::end(newVertices));
    this->indices.insert(std::end(this->indices), std::begin(newIndices),
                         std::end(newIndices));
    this->normals.insert(std::end(this->normals), std::begin(newNormals),
                         std::end(newNormals));

    // populate  triangles
    for (size_t faceID = nFaces() - (newIndices.size() / 3); faceID < nFaces();
         ++faceID) {
      tinyobj::material_t m;

      m.diffuse[0] = color[0];
      m.diffuse[1] = color[1];
      m.diffuse[2] = color[2];
      m.ambient[0] = 0;
      m.ambient[1] = 0;
      m.ambient[2] = 0;
      m.emission[0] = color[0] * (intensity);
      m.emission[1] = color[1] * (intensity);
      m.emission[2] = color[2] * (intensity);
      m.specular[0] = 0.00;
      m.specular[1] = 0.00;
      m.specular[2] = 0.00;
      m.dissolve = 1.0;
      m.illum = 1;

      this->materials.emplace_back(m);

      // populate BxDF
      const auto material = this->materials[faceID];
      if (material) {
        tinyobj::material_t m = material.value();
        this->bxdfs.push_back(createBxDF(m));
      }
      // default material
      else {
        this->bxdfs.push_back(createDefaultBxDF());
      }
    }
  }
```

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/scene.hpp#L335C1-L368C4

```cpp
  void setupTriangles() {
    // populate  triangles
    for (size_t faceID = 0; faceID < nFaces(); ++faceID) {
      // add triangle
      this->triangles.emplace_back(this->vertices.data(), this->indices.data(),
                                   this->normals.data(), faceID);
    }

    // populate lights, primitives
    for (size_t faceID = 0; faceID < nFaces(); ++faceID) {
      // add light
      boost::shared_ptr<Light> light = nullptr;
      const auto material = this->materials[faceID];
      // std::cout << "material check" << std::endl;
      if (material) {
        tinyobj::material_t m = material.value();
        light = createAreaLight(m, &this->triangles[faceID]);
        if (light != nullptr) {
          lights.push_back(light);
        }
      }
      // add primitive
      // std::cout << "Adding primitives" << std::endl;
      primitives.emplace_back(&this->triangles[faceID], this->bxdfs[faceID],
                              light);
    }
    // std::cout << "Triangles setup! " << std::endl;

#ifdef __OUTPUT__
    std::cout << "[Scene] vertices: " << nVertices() << std::endl;
    std::cout << "[Scene] faces: " << nFaces() << std::endl;
    std::cout << "[Scene] lights: " << lights.size() << std::endl;
#endif
  }
```


https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/scene.hpp#L730C1-L777C4

```cpp
  bool intersect(const Ray &ray, IntersectInfo &info) const {
    RTCRayHit rayhit{};
    rayhit.ray.org_x = ray.origin[0];
    rayhit.ray.org_y = ray.origin[1];
    rayhit.ray.org_z = ray.origin[2];
    rayhit.ray.dir_x = ray.direction[0];
    rayhit.ray.dir_y = ray.direction[1];
    rayhit.ray.dir_z = ray.direction[2];
    rayhit.ray.tnear = 0;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = -1;

    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instPrimID[0] = RTC_INVALID_GEOMETRY_ID;

    RTCRayQueryContext context;
    rtcInitRayQueryContext(&context);
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.context = &context;

    rtcIntersect1(scene, &rayhit, &args);

    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      info.t = rayhit.ray.tfar;

      // get triangle shape
      const Triangle &tri = this->triangles[rayhit.hit.primID];

      // set surface info
      info.surfaceInfo.position = ray(info.t);
      info.surfaceInfo.barycentric = Vec2f(rayhit.hit.u, rayhit.hit.v);
      info.surfaceInfo.geometricNormal = tri.getGeometricNormal();
      info.surfaceInfo.shadingNormal =
          tri.computeShadingNormal(info.surfaceInfo.barycentric);
      orthonormalBasis(info.surfaceInfo.shadingNormal, info.surfaceInfo.dpdu,
                       info.surfaceInfo.dpdv);

      // set primitive
      info.hitPrimitive = &this->primitives[rayhit.hit.primID];

      return true;
    } else {
      return false;
    }
  }
```

## 2.4. Carte de Photon

https://github.com/AurelienBesnier/photon_mapping/blob/main/src/cpp/include/photon_map.hpp#L15

```cpp
struct Photon {
    Vec3f throughput;  ///< BxDF * Geometric Term / pdf
    Vec3f position; ///< The position of the photon in the scene
    Vec3f wi;  ///<  incident direction
    unsigned int triId = 0; ///<  id of the triangle on which the photon ended up
};
```

KDTree

## 2.5. Photon Tracing

échantionnage un rayon de lumière

```cpp
// sample initial ray from light and compute initial throughput
static Ray sampleRayFromLight(const Scene &scene, Sampler &sampler, Vec3f &throughput) {
	// sample light
	float light_choose_pdf;
	boost::shared_ptr<Light> light = scene.sampleLight(sampler,
				light_choose_pdf);

	// sample point on light
	float light_pos_pdf;
	SurfaceInfo light_surf = light->samplePoint(sampler, light_pos_pdf);

	// sample direction on light
	float light_dir_pdf;
	Vec3f dir = light->sampleDirection(light_surf, sampler, light_dir_pdf);

	// spawn ray
	Ray ray(light_surf.position, dir);
	Vec3f le = light->Le();

	throughput = le / (light_choose_pdf * light_pos_pdf * light_dir_pdf)
				* std::abs(dot(dir, light_surf.shadingNormal));

	return ray;
}
```

## 2.6. Photon Collecting

calculer le radiance une region

```cpp
	// compute reflected radiance with global photon map
	Vec3f computeRadianceWithPhotonMap(const Vec3f &wo,
			IntersectInfo &info) const {
		// get nearby photons
		float max_dist2;
		const std::vector<int> photon_indices =
				globalPhotonMap.queryKNearestPhotons(info.surfaceInfo.position,
						nEstimationGlobal, max_dist2);

		Vec3f Lo;
		for (const int photon_idx : photon_indices) {
			const Photon &photon = globalPhotonMap.getIthPhoton(photon_idx);
			const Vec3f f = info.hitPrimitive->evaluateBxDF(wo, photon.wi,
					info.surfaceInfo, TransportDirection::FROM_CAMERA);
			Lo += f * photon.throughput;
		}
		if (!photon_indices.empty()) {
			Lo /= (nPhotonsGlobal * PI * max_dist2);
		}
		return Lo;
	}
```

## 2.7. La programme principal

# Reférences
* [1] Bernard Péroche, Dominique Bechmann. Informatique garaphique et rendu. Lavoisier, 2007.
* [2] Schill, Steven & Jensen, John & Raber, George & Porter, Dwayne. (2004). Temporal Modeling of Bidirectional Reflection Distribution Function (BRDF) in Coastal Vegetation. GIScience & Remote Sensing. 41. 116-135. 10.2747/1548-1603.41.2.116. 
* [3] https://opg.optica.org/ao/abstract.cfm?uri=ao-13-1-109
