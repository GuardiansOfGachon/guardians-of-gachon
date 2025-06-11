package com.example.myapplication;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.AudioManager;
import android.os.Build;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.speech.tts.Voice;
import android.util.Base64;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.json.JSONObject;

import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public class Notification_page extends AppCompatActivity {

    private static final String TAG = "WebSocket";

    private RecyclerView recyclerView;
    private NotificationAdapter adapter;
    private List<NotificationItem> notificationList = new ArrayList<>();
    private ImageView imgWarning;

    private WebSocketClient webSocketClient;

    private Map<Long, Bitmap> imageMap = new HashMap<>();
    private Map<Long, String> outputMap = new HashMap<>();

    private TextToSpeech tts;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_notification);

        // RecyclerView 설정
        recyclerView = findViewById(R.id.recyclerViewNotifications);
        recyclerView.setLayoutManager(new LinearLayoutManager(this));
        adapter = new NotificationAdapter(notificationList);
        recyclerView.setAdapter(adapter);

        // 경고 아이콘 View 참조 (레이아웃에 추가 필요)
        imgWarning = findViewById(R.id.imgWarning);

        // TTS 초기화
        tts = new TextToSpeech(this, status -> {
            if (status == TextToSpeech.SUCCESS) {
                int langResult = tts.setLanguage(Locale.ENGLISH);
                if (langResult == TextToSpeech.LANG_MISSING_DATA || langResult == TextToSpeech.LANG_NOT_SUPPORTED) {
                    Log.e(TAG, "TTS language data missing or not supported");
                    Intent installIntent = new Intent(TextToSpeech.Engine.ACTION_INSTALL_TTS_DATA);
                    startActivity(installIntent);
                } else {
                    // 음성 스타일 설정
                    tts.setPitch(0.9f);
                    tts.setSpeechRate(0.9f);
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                        for (Voice v : tts.getVoices()) {
                            if (v.getLocale().equals(Locale.US)
                                    && !v.isNetworkConnectionRequired()
                                    && v.getQuality() >= 400) {
                                tts.setVoice(v);
                                Log.d(TAG, "Selected trustworthy voice: " + v.getName());
                                break;
                            }
                        }
                    }
                    Log.d(TAG, "TTS initialized with style settings");
                }
            } else {
                Log.e(TAG, "TTS initialization failed");
            }
        });

        // 오디오 포커스 요청 및 볼륨 조정
        AudioManager am = (AudioManager) getSystemService(Context.AUDIO_SERVICE);
        int focus = am.requestAudioFocus(null,
                AudioManager.STREAM_MUSIC,
                AudioManager.AUDIOFOCUS_GAIN_TRANSIENT);
        if (focus != AudioManager.AUDIOFOCUS_REQUEST_GRANTED) {
            Log.w(TAG, "Audio focus request failed");
        }
        if (am.getStreamVolume(AudioManager.STREAM_MUSIC) == 0) {
            int max = am.getStreamMaxVolume(AudioManager.STREAM_MUSIC);
            am.setStreamVolume(AudioManager.STREAM_MUSIC, max / 2, 0);
            Log.d(TAG, "STREAM_MUSIC volume set to mid level");
        }

        connectWebSocket();
    }

    private void connectWebSocket() {
        URI uri;
        try {
            uri = new URI("your URL");
            Log.d(TAG, "Connecting to WebSocket: " + uri);
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        webSocketClient = new WebSocketClient(uri) {
            @Override
            public void onOpen(ServerHandshake handshakedata) {
                runOnUiThread(() -> Toast.makeText(Notification_page.this,
                        "Connected to ROS2", Toast.LENGTH_SHORT).show());
                subscribeTopic("/image_with_timestamp", "sensor_msgs/CompressedImage");
                subscribeTopic("/output", "std_msgs/String");
            }

            @Override
            public void onMessage(String message) {
                Log.d(TAG, "WebSocket message: " + message);
                try {
                    JSONObject json = new JSONObject(message);
                    String topic = json.getString("topic");

                    if (topic.equals("/image_with_timestamp")) {
                        JSONObject msg = json.getJSONObject("msg");
                        long ts = msg.getJSONObject("header").getJSONObject("stamp").getLong("sec");
                        String imgData = msg.getString("data");
                        byte[] decoded = Base64.decode(imgData, Base64.DEFAULT);
                        Bitmap bmp = BitmapFactory.decodeByteArray(decoded, 0, decoded.length);
                        imageMap.put(ts, bmp);
                        addNotificationIfMatched(ts);

                    } else if (topic.equals("/output")) {
                        String data = json.getJSONObject("msg").getString("data");
                        String[] parts = data.split("\\|");
                        if (parts.length == 2) {
                            long ts = Long.parseLong(parts[0]);
                            outputMap.put(ts, parts[1]);
                            runOnUiThread(() -> speakIfNeeded(parts[1]));
                            addNotificationIfMatched(ts);
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onClose(int code, String reason, boolean remote) {
                runOnUiThread(() -> Toast.makeText(Notification_page.this,
                        "Disconnected", Toast.LENGTH_SHORT).show());
            }

            @Override
            public void onError(Exception ex) {
                ex.printStackTrace();
            }
        };
        webSocketClient.connect();
    }

    private void speakIfNeeded(String text) {
        String s = text.replace("</s>", "");
        int nl = s.indexOf("\n");
        if (nl != -1) s = s.substring(0, nl);
        if (!s.contains("No Hazard")) {
            tts.speak(s, TextToSpeech.QUEUE_FLUSH, new Bundle(), "ros_output");
        }
    }

    private void showWarningAnimation() {
        runOnUiThread(() -> {
            imgWarning.setAlpha(0f);
            imgWarning.setVisibility(View.VISIBLE);
            imgWarning.animate()
                    .alpha(1f)
                    .setDuration(300)
                    .withEndAction(() -> imgWarning.animate()
                            .alpha(0f)
                            .setStartDelay(800)
                            .setDuration(300)
                            .withEndAction(() -> imgWarning.setVisibility(View.GONE))
                            .start())
                    .start();
        });
    }

    private void addNotificationIfMatched(long timestamp) {
        if (imageMap.containsKey(timestamp) && outputMap.containsKey(timestamp)) {
            String text = outputMap.get(timestamp);
            String display = text.replace("</s>", "");
            if (display.contains("No Hazard")) {
                imageMap.remove(timestamp);
                outputMap.remove(timestamp);
                return;
            }
            showWarningAnimation();
            NotificationItem item = new NotificationItem(display, imageMap.get(timestamp));
            runOnUiThread(() -> {
                adapter.addNotification(item);
                recyclerView.scrollToPosition(0);
            });
            imageMap.remove(timestamp);
            outputMap.remove(timestamp);
        }
    }

    private void subscribeTopic(String topic, String type) {
        try {
            JSONObject json = new JSONObject();
            json.put("op", "subscribe");
            json.put("topic", topic);
            json.put("type", type);
            webSocketClient.send(json.toString());
        } catch (Exception e) {
            Log.e(TAG, "Subscribe failed: " + topic, e);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (tts != null) {
            tts.stop();
            tts.shutdown();
        }
        if (webSocketClient != null) webSocketClient.close();
    }
}
