void view_tracks(downward_target_tracker::info info, int selected_id)
{
    vdbOrtho(0.0f, info.image_x, info.image_y, 0.0f);
    for (int i = 0; i < info.num_targets; i++)
    {
        // last seen center and bounding box
        float u = info.last_seen_u[i];
        float v = info.last_seen_v[i];
        float u1 = info.last_seen_u1[i];
        float v1 = info.last_seen_v1[i];
        float u2 = info.last_seen_u2[i];
        float v2 = info.last_seen_v2[i];

        glPoints(14.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
        glVertex2f(u, v);
        glEnd();

        glLines(2.0f);
        if (selected_id == info.unique_id[i])
            glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
        else
            glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
        glVertex2f(u1, v1); glVertex2f(u2, v1);
        glVertex2f(u2, v1); glVertex2f(u2, v2);
        glVertex2f(u2, v2); glVertex2f(u1, v2);
        glVertex2f(u1, v2); glVertex2f(u1, v1);
        glEnd();

        vdbNote(u, v, "ID: %d", info.unique_id[i]);
    }
}
