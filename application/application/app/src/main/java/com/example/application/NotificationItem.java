package com.example.myapplication;

import android.graphics.Bitmap;

public class NotificationItem {
    private String text;
    private Bitmap image;

    public NotificationItem(String text, Bitmap image) {
        this.text = text;
        this.image = image;
    }

    public String getText() {
        return text;
    }

    public Bitmap getImage() {
        return image;
    }
}
