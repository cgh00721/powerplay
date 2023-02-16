package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.ReaderException;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;

import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.IOException;
import java.util.Hashtable;
import java.util.Map;


public class BarcodeDetectionPipeline extends OpenCvPipeline {

    public static int latestResult = 030;

    public static void detectBarcode(Mat mat) throws IOException, NotFoundException {
        Bitmap bitmap = convertMatToBitMap(mat);
        MultiFormatReader multiFormatReader = new MultiFormatReader();
        Map<DecodeHintType, Object> hints = new Hashtable<>();
        hints.put(DecodeHintType.PURE_BARCODE, Boolean.TRUE);
        multiFormatReader.setHints(hints);

        int width = bitmap.getWidth();
        int height = bitmap.getHeight();

        int[] pixels = new int[width * height];
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);

        Result rawResult = null;
        RGBLuminanceSource source = new RGBLuminanceSource(width, height, pixels);

        if (source != null) {
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            try {
                rawResult = multiFormatReader.decodeWithState(binaryBitmap);
            } catch (ReaderException re) {
                re.printStackTrace();
            } finally {
                multiFormatReader.reset();
            }
        }
        GlobalTelemetry.telemetry.addData("RAW:",rawResult);

        if (rawResult != null) {
            latestResult = Integer.parseInt(rawResult.getText());
        }
    }

    public static Bitmap convertMatToBitMap(Mat input) {
        Bitmap bmp = null;
        Mat rgb = new Mat();
        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_BGR2RGB);

        try {
            bmp = Bitmap.createBitmap(rgb.cols(), rgb.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgb, bmp);
        }
        catch (CvException e){
            Log.d("Exception",e.getMessage());
        }
        return bmp;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            detectBarcode(input);
        } catch (Exception e) {
            GlobalTelemetry.telemetry.addData("Error:",e);
            e.printStackTrace();
        }
        return input;
    }

    public static int getLatestResult() {
        return latestResult;
    }
}