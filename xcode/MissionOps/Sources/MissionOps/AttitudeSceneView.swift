import AppKit
import SceneKit
import SpacecraftSim
import SwiftUI

/// SceneKit 3D attitude view: the solid spacecraft chasing its target
/// attitude, shown as an orange wireframe ghost. Drag to orbit the camera.
struct AttitudeSceneView: NSViewRepresentable {
    var attitude: Quat  // estimated attitude, body -> inertial
    var target: Quat    // target attitude, body -> inertial

    func makeNSView(context: Context) -> SCNView {
        let view = SCNView()
        view.backgroundColor = .black
        view.allowsCameraControl = true
        view.autoenablesDefaultLighting = true

        let scene = SCNScene()

        // Spacecraft: bus + two solar wings + antenna mast.
        let craft = SCNNode()
        let busMat = SCNMaterial()
        busMat.diffuse.contents = NSColor(white: 0.72, alpha: 1.0)
        busMat.metalness = 0.7
        busMat.roughness = 0.35
        let bus = SCNNode(geometry: SCNBox(width: 0.5, height: 0.5, length: 0.7, chamferRadius: 0.02))
        bus.geometry?.materials = [busMat]
        craft.addChildNode(bus)

        let panelMat = SCNMaterial()
        panelMat.diffuse.contents = NSColor.systemBlue
        panelMat.metalness = 0.3
        panelMat.roughness = 0.5
        for side in [-1.0, 1.0] {
            let panel = SCNNode(geometry: SCNBox(width: 0.9, height: 0.02, length: 0.5, chamferRadius: 0))
            panel.geometry?.materials = [panelMat]
            panel.position = SCNVector3(side * 0.72, 0, 0)
            craft.addChildNode(panel)
        }
        let mast = SCNNode(geometry: SCNCylinder(radius: 0.02, height: 0.4))
        mast.position = SCNVector3(0, 0.45, 0)
        craft.addChildNode(mast)
        scene.rootNode.addChildNode(craft)

        // Target attitude ghost: wireframe box, same proportions, slightly larger.
        let ghostMat = SCNMaterial()
        ghostMat.diffuse.contents = NSColor.systemOrange
        ghostMat.fillMode = .lines
        ghostMat.isDoubleSided = true
        let ghost = SCNNode(geometry: SCNBox(width: 0.56, height: 0.56, length: 0.76, chamferRadius: 0.02))
        ghost.geometry?.materials = [ghostMat]
        scene.rootNode.addChildNode(ghost)

        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(2.2, 1.6, 2.8)
        cameraNode.look(at: SCNVector3Zero)
        scene.rootNode.addChildNode(cameraNode)

        context.coordinator.craft = craft
        context.coordinator.ghost = ghost
        view.scene = scene
        return view
    }

    func updateNSView(_ nsView: SCNView, context: Context) {
        // SCNNode.orientation is (x, y, z, w), scalar-last — same as Quat.
        // The parent frame is the inertial frame, so this renders the true body orientation.
        context.coordinator.craft?.orientation = scnQuaternion(attitude)
        context.coordinator.ghost?.orientation = scnQuaternion(target)
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    final class Coordinator {
        var craft: SCNNode?
        var ghost: SCNNode?
    }

    private func scnQuaternion(_ q: Quat) -> SCNVector4 {
        SCNVector4(Float(q.x), Float(q.y), Float(q.z), Float(q.w))
    }
}
