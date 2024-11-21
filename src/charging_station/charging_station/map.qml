import QtQuick 2.15
import QtQuick.Controls 2.15
import QtLocation 5.15
import QtPositioning 5.15

Item {
    width: 800
    height: 600

    Plugin {
        id: mapPlugin
        name: "osm"  // OpenStreetMap como proveedor
    }

    Map {
        anchors.fill: parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(37.7749, -122.4194)  // Coordenadas iniciales
        zoomLevel: 14

        // Marcador del dron
        MapQuickItem {
            coordinate: QtPositioning.coordinate(37.7749, -122.4194)  // Coordenadas del dron
            anchorPoint.x: 12
            anchorPoint.y: 12
            sourceItem: Rectangle {
                width: 24
                height: 24
                color: "blue"
                radius: 12
                border.color: "white"
                border.width: 2
            }
        }
    }
}
