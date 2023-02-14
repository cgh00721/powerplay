/*package org.firstinspires.ftc.teamcode;

import com.google.zxing.*;
import com.google.zxing.client.j2se.BufferedImageLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
//import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

//import javax.imageio.ImageIO;
//import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
public class Barcode {

    public static void processFrame(Mat mat) throws IOException, Exception {
        BufferedImage image = Mat2BufferedImage(mat);
        LuminanceSource source = new BufferedImageLuminanceSource(image);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
        Result result = new MultiFormatReader().decode(bitmap);
        System.out.println("Barcode text is: " + result.getText());
    }

    public static BufferedImage Mat2BufferedImage(Mat mat) throws IOException{
        //Encoding the image
        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".jpg", mat, matOfByte);
        //Storing the encoded Mat in a byte array
        byte[] byteArray = matOfByte.toArray();
        //Preparing the Buffered Image
        InputStream in = new ByteArrayInputStream(byteArray);
        return ImageIO.read(in);
    }

    public static void main(String[] args)  {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Mat mat = new Mat();
        VideoCapture capture = new VideoCapture(0);
        while (true) {
            capture.read(mat);
            HighGui.imshow("Barcode", mat);
            HighGui.waitKey(1);
            try {
                processFrame(mat);
            } catch (IOException | NotFoundException e) {}
        }
    }
}
*/