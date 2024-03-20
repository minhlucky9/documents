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

## 2.1. Matériel
## 2.2. Lumière
## 2.3. Scène
## 2.4. Carte de Photon
## 2.5. Photon Tracing
## 2.6. Photon Collecting

# Reférences
* Bernard Péroche, Dominique Bechmann. Informatique garaphique et rendu. Lavoisier, 2007.
* Schill, Steven & Jensen, John & Raber, George & Porter, Dwayne. (2004). Temporal Modeling of Bidirectional Reflection Distribution Function (BRDF) in Coastal Vegetation. GIScience & Remote Sensing. 41. 116-135. 10.2747/1548-1603.41.2.116. 