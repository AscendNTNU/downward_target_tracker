void view_tracks(downward_target_tracker::info info, downward_target_tracker::tracks tracks, int selected_id)
{
    vdbOrtho(0.0f, tracks.image_x, tracks.image_y, 0.0f);
    for (int i = 0; i < tracks.num_targets; i++)
    {
        // filtered position in image
        float u_hat = tracks.u_hat[i];
        float v_hat = tracks.v_hat[i];

        // last seen center and bounding box
        float u = tracks.u[i];
        float v = tracks.v[i];
        float u1 = tracks.u1[i];
        float v1 = tracks.v1[i];
        float u2 = tracks.u2[i];
        float v2 = tracks.v2[i];

        glPoints(8.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glVertex2f(u_hat, v_hat);
        glEnd();

        glPoints(14.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
        glVertex2f(u, v);
        glEnd();

        glLines(2.0f);
        if (selected_id == tracks.unique_id[i])
            glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
        else
            glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
        glVertex2f(u1, v1); glVertex2f(u2, v1);
        glVertex2f(u2, v1); glVertex2f(u2, v2);
        glVertex2f(u2, v2); glVertex2f(u1, v2);
        glVertex2f(u1, v2); glVertex2f(u1, v1);
        glEnd();

        vdbNote(u_hat, v_hat, "ID: %d", tracks.unique_id[i]);
    }
}
