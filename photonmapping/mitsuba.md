## 1. Introduction

Mitsuba est un système de rendu orienté recherche pour la simulation de la lumière. Durant la développement de ce moteur, il y a 3 versions principaux, ce sont Mitsuba 0.6, Mitsuba 2 et Mitsuba 3. Pourtant, en fait, chaque version est un complètement nouveau système qui poursuit un emsemble d'objetifs différents.

### 1.1. Mitsuba 0.6

**Mitsuba 0.6** est la première version de Mitsuba. Il contiens plus de 100 modules qui implémente des fonctionnalités de matériels, de sources de lumières et d'algorithmes de rendu. Toutes ces modules sont écrit complètement en C++. Et ce moteur fonctionne seulement sur CPU. 

L'objectif principal de cette version est de construire un moteur qui rend des images le plus réelle. Alors, il a plusieurs modèles de réflexion basés à la physique. Ce moteur est très util à explorer et mieux comprendre le principe de base des moteurs de jeux courant. 

### 1.2. Mitsuba 2

Dans la deuxième version, le plus important changement est de faire Mitsuba devenir un *retargetable renderer*. C'est-à-dire que ce moteur peut être utilisé pour faire plusieurs taches:

* Faire le rendu d'image sur CPU comme la première version
* Supporter le SIMD de CPU modern pour échantillonner des rayons de lumière en parallel
* Devenir un moteur de rendu différenciable qui fonctionne sur NDIVIA RTX GPUs. Un algorithme de rendu différenciable est un algorithme basé sur Machine Learning, il va transformer des paramètres la scène tels ques: la position de caméra, geopetrie, BRDF, etc à une image (3D à 2D) et faire l'inverse (2D à 3D).

D'autre part, Mitsuba 2 utilise la bibliothèque *pybind11* pour transformer des fonctions C++ en Python pour qu'on peut utiliser ces fonctions et visualiser le résultat sur Jupyter Notebook.

### 1.3. Mitsuba 3

**Mitsuba 3** est profondément intégré à Python. C'est-à-dire tous les algorithme de de matériels, textures et rendu peut être implémenté en Python. Cela nous permets de réaliser les recherches de la cominaison entre la graphique d'ordinator et l'intelligence artificielle plus facile. 

Ce moteur est un système de rendu différenciable basé sur un algorithme de rendu différenciable développé en EPFL. Il supporte du ray tracing sur GPU avec le bibliothèque CUDE/OptiX.

## 2. Usage

### 2.1. Rendu d'image

Tous les informations de matériels, BSDFs, lumières vont être défini sur un ficher .xml.

```xml
<scene version="3.0.0">
    <default name="spp" value="128"/>
    <default name="res" value="256"/>
    <default name="max_depth" value="6"/>
    <default name="integrator" value="path"/>

    <bsdf type="dielectric" id="glass"/>

    <shape type="sphere" id="glasssphere">
        <transform name="to_world">
            <scale value="0.25"/>
            <translate x="0.5" y="-0.75" z="-0.2"/>
        </transform>
        <ref id="glass"/>
    </shape>

</scene>
```

Ensuite, on va télécharger ce ficher par Mitsuba et faire le rendu d'image avec la fonction intégrée.

```python
import drjit as dr
import mitsuba as mi
mi.set_variant('llvm_ad_rgb')

scene = mi.load_file("../scenes/simple.xml")
original_image = mi.render(scene, spp=128)
```

D'autre part, il est aussi possible d'implémenter notre renderer avec les fonctions de base de ce moteur.

### 2.2. Photon Mapping

L'algorithme Photon Mapping est implémenté seulement sur la version **Mitsuba 0.6**. Les deux dernière version ne supporte pas cet algorithme. Alors, si on veut utiliser Mitsuba pour faire notre simulation d'interception de lumière, il faut de travailler avec la version 0.6 ou construire le photon mapping à partir des fonctions de base des nouvelles versions.

Dans la prémier version, il nous donne une fonction pour appliquer le photon mapping pour faire le rendu d'image. Alors, si on voudrais accèder au photon map ou compter des photons, il faut de modifier le source code de la bibliothque en C++. Cette tache n'est pas facile car le source code est vraiment très compliqué.

D'autre part, dans la prémier version, toutes les taches de lancer des rayons sont marché sur CPU, alors la performance n'est pas très intéressant. 

