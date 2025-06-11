package com.example.myapplication;

import android.graphics.Bitmap;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import java.util.List;

public class NotificationAdapter extends RecyclerView.Adapter<NotificationAdapter.NotificationViewHolder> {

    private List<NotificationItem> notificationList;

    public NotificationAdapter(List<NotificationItem> notificationList) {
        this.notificationList = notificationList;
    }

    @NonNull
    @Override
    public NotificationViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        View view = LayoutInflater.from(parent.getContext()).inflate(R.layout.item_notification, parent, false);
        return new NotificationViewHolder(view);
    }

    @Override
    public void onBindViewHolder(@NonNull NotificationViewHolder holder, int position) {
        NotificationItem item = notificationList.get(position);
        holder.textViewNotification.setText(item.getText());
        holder.imageViewNotification.setImageBitmap(item.getImage());
    }

    @Override
    public int getItemCount() {
        return notificationList.size();
    }

    public void addNotification(NotificationItem item) {
        notificationList.add(0, item);  // 최신 알림을 맨 위에 추가
        notifyItemInserted(0);
    }

    public static class NotificationViewHolder extends RecyclerView.ViewHolder {
        ImageView imageViewNotification;
        TextView textViewNotification;

        public NotificationViewHolder(@NonNull View itemView) {
            super(itemView);
            imageViewNotification = itemView.findViewById(R.id.imageViewNotification);
            textViewNotification = itemView.findViewById(R.id.textViewNotification);
        }
    }
}
