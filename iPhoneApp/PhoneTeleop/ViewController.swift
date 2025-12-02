//
//  ViewController.swift
//  PhoneTeleop
//
//  Created by Chakshan Kothakota on 2/4/25.
//

import UIKit
import SceneKit
import ARKit
import Network

class ViewController: UIViewController, ARSCNViewDelegate, ARSessionDelegate {

    @IBOutlet var sceneView: ARSCNView!
    
    var connection: NWConnection?
    let SERVER_IP = ProcessInfo.processInfo.environment["SERVER_IP"] ?? ""
    let PORT: UInt16 = {
            if let envPort = ProcessInfo.processInfo.environment["PORT"],
               let portValue = UInt16(envPort) {
                return portValue
            }
            return 0
        }()
    
    var count = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        sceneView.session.delegate = self
        
        // Show statistics such as fps and timing information
        sceneView.showsStatistics = true
        
        startUDPBroadcast()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()

        // Run the view's session
        sceneView.session.run(configuration)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    func startUDPBroadcast() {
        let host = NWEndpoint.Host(SERVER_IP)
        guard let port = NWEndpoint.Port(rawValue: PORT) else {
            print("Invalid port number")
            return
        }
        connection = NWConnection(host: host, port: port, using: .udp)
        connection?.stateUpdateHandler = { state in
            switch state {
            case .ready:
                print("UDP Client ready")
                break
            case .failed(let error):
                print("Client failed with error: \(error)")
                break
            default:
                print("Connection state: \(state)")
                break
            }
        }
        connection?.start(queue: .global())
        print("UDP broadcasting started to \(SERVER_IP):\(PORT)")
    }
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let transform = frame.camera.transform
        let tfMatrix: [[Float]] = [
            [transform.columns.0.x, transform.columns.1.x, transform.columns.2.x, transform.columns.3.x],
            [transform.columns.0.y, transform.columns.1.y, transform.columns.2.y, transform.columns.3.y],
            [transform.columns.0.z, transform.columns.1.z, transform.columns.2.z, transform.columns.3.z],
            [transform.columns.0.w, transform.columns.1.w, transform.columns.2.w, transform.columns.3.w]
        ]
        
        let jsonData : [String : [[Float]]] = [
            "transform" : tfMatrix
        ]
        
        if let data = try? JSONSerialization.data(withJSONObject: jsonData) {
            self.connection?.send(content: data, completion: .contentProcessed({error in
                if let error = error {
                    print("Error sending JSON data: \(error)")
                }
            }))
        }
    }

}
