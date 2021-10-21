package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.icu.util.Output;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.View;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.OutputStream;

public class FuckingViewPort{
    private LinearLayout viewportContainerLayout;
    private SurfaceView view;

    public FuckingViewPort(int containerLayoutId) {
        setupViewport(containerLayoutId);

        // AppUtil.getInstance().getApplication().registerComponentCallbacks();
        // OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).registerListener();
    }

    private void setupViewport(final int containerLayoutId) {
        AppUtil.getInstance().getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                try {
                    viewportContainerLayout = (LinearLayout) AppUtil.getInstance().getActivity().findViewById(containerLayoutId);

                    if(viewportContainerLayout == null) {
                        throw new Exception("Viewport container specified by user does not exist");
                    } else if(viewportContainerLayout.getChildCount() != 0) {
                        throw new Exception("Viewport container specified by user is not empty!");
                    }

                    view = new SurfaceView(AppUtil.getInstance().getActivity());
                    viewportContainerLayout.setVisibility(View.VISIBLE);
                    viewportContainerLayout.addView(view);
                } catch (Exception e) { e.printStackTrace(); }
            }
        });
    }

    public void draw(Bitmap currentBitmap)
    {
        while (true)
        {
            Canvas canvas = view.getHolder().lockCanvas();

            if (canvas != null)
            {
                if(currentBitmap != null)
                {
                    Bitmap newBitmap = currentBitmap.copy(currentBitmap.getConfig(), true);

                    Paint paint = new Paint();

                    canvas.drawBitmap(newBitmap, 0, 0, paint);

                    newBitmap.recycle();
                }
            }
            
            view.getHolder().unlockCanvasAndPost(canvas);
        }
    }
}
