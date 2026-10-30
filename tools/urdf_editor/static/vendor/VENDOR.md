# Librerias vendoreadas (sin CDN en tiempo de ejecucion)

Descargadas el 2026-10-30 desde el registro npm (tarball oficial), con integridad
`sha512` verificada contra el campo `dist.integrity` del registro. Solo se copiaron
los archivos necesarios, sin modificaciones.

| Libreria | Version | Licencia | Origen | Archivos |
| --- | --- | --- | --- | --- |
| three.js | 0.186.1 | MIT (`three/LICENSE`) | https://registry.npmjs.org/three/-/three-0.186.1.tgz | `build/three.module.js`, `build/three.core.js`, `examples/jsm/controls/OrbitControls.js`, `examples/jsm/loaders/{STLLoader,ColladaLoader,TGALoader}.js`, `examples/jsm/loaders/collada/*` |
| urdf-loader | 0.13.1 | **Apache-2.0** (`urdf-loader/LICENSE`) | https://registry.npmjs.org/urdf-loader/-/urdf-loader-0.13.1.tgz | `src/URDFLoader.js`, `src/URDFClasses.js` |

Nota de licencia: el brief de HOME-04 pedia documentar "licencia MIT"; `urdf-loader`
es **Apache-2.0** segun su `package.json` y el repositorio upstream
(https://github.com/gkjohnson/urdf-loaders). El tarball npm no incluye el archivo
LICENSE; se copio el texto de `LICENSE` del repositorio upstream (rama `master`).
Apache-2.0 es compatible con este uso (redistribucion con aviso de licencia).

SHA-256 de los archivos copiados:

```
9edde002b066a9a05676a6127f67735b62baf399bdea529f2f7e31657da769e6  three/build/three.core.js
9052042d676cb0fdc1ddfefe193053f34b7ac0513a616fdac4535d49987812ea  three/build/three.module.js
3d79d07ecb686b4e5d93232eedab255331c1beef711e13164eaa1f68655a5f2b  three/examples/jsm/controls/OrbitControls.js
d96f4dcf36bc6a30a4d385bce511ed10e27893d1824074daa97ac863df1232e8  three/examples/jsm/loaders/collada/ColladaComposer.js
d9532ab17f1d9e2919deba8029d759d5dcaed2f21195ded9e29491f719b1a0c6  three/examples/jsm/loaders/collada/ColladaParser.js
b150c67c4bf98ed1eec3cf291e1bd00fc53016488bcefadd7a24f659e86e52d9  three/examples/jsm/loaders/ColladaLoader.js
023ed97f848b633d8bcd53d4db3b996d29d0c644088700691297c552257d480b  three/examples/jsm/loaders/STLLoader.js
da4cfa0c050b8076a63596f36f6e5a54a4a8271774a5301482996be030505c43  three/examples/jsm/loaders/TGALoader.js
ff7578fc62d0933296a2ca30d2bdb7a856f91871f58fe7857dda4685da45bada  urdf-loader/src/URDFClasses.js
51a42cfedcebe1891791f7d0935b5c47ac7d3c9c7d30641b831029d5608bf8ef  urdf-loader/src/URDFLoader.js
```

Para actualizar: descargar el nuevo tarball, verificar `dist.integrity`, reemplazar
los mismos archivos y actualizar esta tabla y los hashes.
