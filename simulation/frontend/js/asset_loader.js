/**
 * BANTAY-SUNOG 3D Asset Loader
 * Allows the user to load custom .glb / .gltf / .obj 3D robot models and textures
 * with real-time calibration for scale, position, and orientation.
 */

class AssetLoader {
  constructor(robotModel) {
    this.robot = robotModel;
    this.gltfLoader = (typeof THREE.GLTFLoader !== 'undefined') ? new THREE.GLTFLoader() : null;
    this.objLoader = (typeof THREE.OBJLoader !== 'undefined') ? new THREE.OBJLoader() : null;

    this.loadedModel = null;
    this.calibration = {
      scale: 1.0,
      positionOffset: new THREE.Vector3(0, 0, 0),
      rotationOffset: new THREE.Euler(0, 0, 0)
    };
  }

  /**
   * Load a 3D model file from a URL or local path.
   */
  loadFromUrl(url, onProgress, onSuccess, onError) {
    const extension = url.split('.').pop().toLowerCase();

    if (extension === 'glb' || extension === 'gltf') {
      if (!this.gltfLoader) {
        if (onError) onError(new Error("GLTFLoader is not available."));
        return;
      }

      this.gltfLoader.load(
        url,
        (gltf) => {
          this.applyLoadedScene(gltf.scene || gltf.scenes[0]);
          if (onSuccess) onSuccess(this.loadedModel);
        },
        onProgress,
        onError
      );
    } else if (extension === 'obj') {
      if (!this.objLoader) {
        if (onError) onError(new Error("OBJLoader is not available."));
        return;
      }

      this.objLoader.load(
        url,
        (obj) => {
          this.applyLoadedScene(obj);
          if (onSuccess) onSuccess(this.loadedModel);
        },
        onProgress,
        onError
      );
    } else {
      if (onError) onError(new Error(`Unsupported model format: .${extension}`));
    }
  }

  /**
   * Load directly from a browser File object (drag and drop or file input).
   */
  loadFromFile(file, onSuccess, onError) {
    const filename = file.name.toLowerCase();
    const reader = new FileReader();

    if (filename.endsWith('.glb')) {
      reader.onload = (e) => {
        const contents = e.target.result;
        if (!this.gltfLoader) return;
        this.gltfLoader.parse(
          contents,
          '',
          (gltf) => {
            this.applyLoadedScene(gltf.scene || gltf.scenes[0]);
            if (onSuccess) onSuccess(this.loadedModel);
          },
          onError
        );
      };
      reader.readAsArrayBuffer(file);
    } else if (filename.endsWith('.gltf') || filename.endsWith('.obj')) {
      reader.onload = (e) => {
        const text = e.target.result;
        if (filename.endsWith('.gltf') && this.gltfLoader) {
          this.gltfLoader.parse(
            text,
            '',
            (gltf) => {
              this.applyLoadedScene(gltf.scene || gltf.scenes[0]);
              if (onSuccess) onSuccess(this.loadedModel);
            },
            onError
          );
        } else if (filename.endsWith('.obj') && this.objLoader) {
          const obj = this.objLoader.parse(text);
          this.applyLoadedScene(obj);
          if (onSuccess) onSuccess(this.loadedModel);
        }
      };
      reader.readAsText(file);
    } else {
      if (onError) onError(new Error("Please upload a .glb, .gltf, or .obj file."));
    }
  }

  applyLoadedScene(modelScene) {
    // Enable shadows on all child meshes
    modelScene.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });

    this.loadedModel = modelScene;
    this.robot.replaceWithCustomMesh(this.loadedModel, this.calibration);
  }

  updateCalibration(scale, posX, posY, posZ, rotXDeg, rotYDeg, rotZDeg) {
    this.calibration.scale = scale;
    this.calibration.positionOffset.set(posX, posY, posZ);
    this.calibration.rotationOffset.set(
      THREE.MathUtils.degToRad(rotXDeg),
      THREE.MathUtils.degToRad(rotYDeg),
      THREE.MathUtils.degToRad(rotZDeg)
    );

    if (this.loadedModel) {
      this.loadedModel.scale.set(scale, scale, scale);
      this.loadedModel.position.copy(this.calibration.positionOffset);
      this.loadedModel.rotation.copy(this.calibration.rotationOffset);
    }
  }

  resetToDefault() {
    this.robot.useProceduralModel();
    this.loadedModel = null;
  }
}
