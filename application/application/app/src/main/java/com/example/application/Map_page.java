package com.example.myapplication;

import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.json.JSONObject;

import java.net.URI;

public class Map_page extends AppCompatActivity {

    private static final String TAG = "WebSocket"; // Logcat 태그 추가
    private MapView mapView;
    private GoogleMap googleMap;

    private WebSocketClient webSocketClient;

    private Marker vehicleMarker;
    private boolean isFirstLocation = true; // 첫 위치일 때만 줌인

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_map);

        mapView = findViewById(R.id.mapView);
        mapView.onCreate(savedInstanceState);

        mapView.getMapAsync(map -> {
            googleMap = map;

            // 지도 설정
            googleMap.getUiSettings().setZoomControlsEnabled(true);
            googleMap.getUiSettings().setMyLocationButtonEnabled(true);

            // WebSocket 연결
            connectWebSocket();
        });
    }

    private void connectWebSocket() {
        URI uri;
        try {
            uri = new URI("your URL"); // Jetson Orin Nano의 IP 및 포트
            Log.d(TAG, "Connecting to WebSocket: " + uri.toString()); // WebSocket 연결 시 로그 출력
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        webSocketClient = new WebSocketClient(uri) {
            @Override
            public void onOpen(ServerHandshake handshakedata) {
                runOnUiThread(() -> Toast.makeText(Map_page.this, "Connected to ROS2", Toast.LENGTH_SHORT).show());
                Log.d(TAG, "WebSocket connection opened"); // 연결 성공 시 로그 출력

                // WebSocket 연결 후 ROS2 토픽 구독 요청
                subscribeTopic("/vehicle_location", "std_msgs/String");
            }

            @Override
            public void onMessage(String message) {
                Log.d(TAG, "WebSocket message received: " + message); // 메시지 로그 출력
                try {
                    JSONObject json = new JSONObject(message);
                    String topic = json.getString("topic");

                    Log.d(TAG, "Topic: " + topic); // 토픽 로그 출력

                    // 차량 위치 메시지 처리
                    if (topic.equals("/vehicle_location")) {
                        Log.d(TAG, "Vehicle location message received: " + message);
                        JSONObject msg = json.getJSONObject("msg");

                        // 중첩된 "data" 문자열을 파싱
                        String dataString = msg.getString("data");
                        JSONObject data = new JSONObject(dataString);

                        double latitude = data.getDouble("latitude");
                        double longitude = data.getDouble("longitude");

                        runOnUiThread(() -> updateVehicleLocation(latitude, longitude));
                    }
                } catch (Exception e) {
                    Log.e(TAG, "Error processing WebSocket message", e);
                    e.printStackTrace();
                }
            }

            @Override
            public void onClose(int code, String reason, boolean remote) {
                runOnUiThread(() -> Toast.makeText(Map_page.this, "Disconnected from ROS2", Toast.LENGTH_SHORT).show());
                Log.w(TAG, "WebSocket connection closed: " + reason + " (Code: " + code + ")"); // 연결 종료 시 로그 출력
            }

            @Override
            public void onError(Exception ex) {
                ex.printStackTrace();
                runOnUiThread(() -> Toast.makeText(Map_page.this, "Connection error: " + ex.getMessage(), Toast.LENGTH_SHORT).show());
                Log.e(TAG, "WebSocket connection error", ex); // 연결 오류 시 로그 출력
            }
        };

        webSocketClient.connect();
    }

    // WebSocket 연결 후 ROS2 토픽을 구독하는 메소드
    private void subscribeTopic(String topic, String type) {
        try {
            JSONObject subscribeJson = new JSONObject();
            subscribeJson.put("op", "subscribe");
            subscribeJson.put("topic", topic);
            subscribeJson.put("type", type);
            webSocketClient.send(subscribeJson.toString());
            Log.d(TAG, "Subscribed to topic: " + topic); // 구독 성공 시 로그 출력
        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG, "Failed to subscribe to topic: " + topic, e); // 구독 실패 시 로그 출력
        }
    }

    private void updateVehicleLocation(double latitude, double longitude) {
        LatLng vehiclePosition = new LatLng(latitude, longitude);

        if (vehicleMarker == null) {
            vehicleMarker = googleMap.addMarker(new MarkerOptions()
                    .position(vehiclePosition)
                    .title("Vehicle")
                    .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_BLUE)));
            Log.d(TAG, "Vehicle marker added at: " + latitude + ", " + longitude); // 차량 마커 추가 로그 출력
        } else {
            vehicleMarker.setPosition(vehiclePosition);
            Log.d(TAG, "Vehicle marker updated to: " + latitude + ", " + longitude); // 차량 마커 업데이트 로그 출력
        }

        if (isFirstLocation) {
            googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(vehiclePosition, 17));
            isFirstLocation = false;
            Log.d(TAG, "Camera moved to vehicle location (first location)"); // 첫 위치 줌인 시 로그 출력
        } else {
            googleMap.moveCamera(CameraUpdateFactory.newLatLng(vehiclePosition));
            Log.d(TAG, "Camera moved to vehicle location"); // 차량 위치로 카메라 이동 시 로그 출력
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        mapView.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mapView.onPause();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mapView.onDestroy();
        if (webSocketClient != null) {
            webSocketClient.close();
            Log.d(TAG, "WebSocket connection closed"); // WebSocket 연결 종료 시 로그 출력
        }
    }
}
