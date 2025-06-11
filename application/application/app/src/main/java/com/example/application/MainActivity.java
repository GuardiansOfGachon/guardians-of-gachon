package com.example.myapplication;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.ImageView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    private ImageView notificationButton, homeButton, profileButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // 네비게이션 바 버튼 초기화
        notificationButton = findViewById(R.id.notification_button);
        homeButton = findViewById(R.id.home_button);
        profileButton = findViewById(R.id.profile_button);

        // 버튼 클릭 이벤트 설정
        notificationButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                Toast.makeText(MainActivity.this, "알림 페이지 이동", Toast.LENGTH_SHORT).show();
                // 알림 페이지로 이동하려면 Intent 추가 필요

                // Create an Intent to navigate to NotificationActivity
                Intent intent = new Intent(MainActivity.this, Notification_page.class);
                startActivity(intent);
            }
        });

        // 홈 버튼 클릭 시 MainActivity로 돌아가기
        homeButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                Toast.makeText(MainActivity.this, "홈 화면으로 이동", Toast.LENGTH_SHORT).show();
                Intent intent = new Intent(MainActivity.this, MainActivity.class);
                intent.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_SINGLE_TOP);
                startActivity(intent);
            }
        });

        profileButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Toast.makeText(MainActivity.this, "지도 이동", Toast.LENGTH_SHORT).show();
                // 프로필 페이지로 이동하려면 Intent 추가 필요
                Intent intent = new Intent(MainActivity.this, Map_page.class);
                startActivity(intent);
            }
        });
    }
}
