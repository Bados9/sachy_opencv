import org.opencv.core.Mat;

import static org.opencv.highgui.HighGui.imshow;
import static org.opencv.highgui.HighGui.waitKey;

public class ImageReformator {

    public void doIt(Mat image){
        imshow("aa", image);
        waitKey();
    }
}
