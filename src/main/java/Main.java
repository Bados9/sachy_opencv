import javafx.collections.ArrayChangeListener;
import javafx.util.Pair;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.io.File;
import java.lang.reflect.Array;
import java.util.*;

import static org.opencv.calib3d.Calib3d.findHomography;
import static org.opencv.highgui.HighGui.*;
import static org.opencv.imgcodecs.Imgcodecs.*;
import static org.opencv.imgproc.Imgproc.*;

public class Main {

//    public static Mat contourCluster(Mat lines, double rho_max, double theta_max)
//    {
//        Mat combinedLines = new Mat();
//        Vector<Integer>[] combineIndex = new Vector<Integer>((int) lines.total());
//
//        for (int i = 0; i < lines.total(); i++)
//        {
//            int index = i;
//            for (int j = i; j < lines.total(); j++)
//            {
//                double distanceI = lines.get(i/lines.cols(), i%lines.cols())[0], distanceJ = lines.get(j/lines.cols(), j%lines.cols())[0];
//                double slopeI = lines.get(i/lines.cols(), i%lines.cols())[1], slopeJ = lines.get(j/lines.cols(), j%lines.cols())[1];
//                double disDiff = Math.abs(distanceI - distanceJ);
//                double slopeDiff = Math.abs(slopeI - slopeJ);
//
//                if (slopeDiff < theta_max && disDiff < rho_max)
//                {
//                    boolean isCombined = false;
//                    for (int w = 0; w < i; w++)
//                    {
//                        for (int u = 0; u < combineIndex[w].length; u++)
//                        {
//                            if (combineIndex[w][u] == j)
//                            {
//                                isCombined = true;
//                                break;
//                            }
//                            if (combineIndex[w][u] == i)
//                                index = w;
//                        }
//                        if (isCombined)
//                            break;
//                    }
//                    if (!isCombined)
//                        combineIndex[index].add(j);
//                }
//            }
//        }
//
//        for (int i = 0; i < combineIndex.size(); i++)
//        {
//            if (combineIndex.get(i).size() == 0)
//                continue;
//            Mat line_temp = new Mat();
//            for (int j = 0; j < combineIndex.get(i).size(); j++) {
//                int i1 = combineIndex.get(i).get(j);
//                double[] a = line_temp.get(0, 0);
//                a[0] += lines.get(i1/lines.cols(), i1%lines.cols())[0];
//
//                double[] b = line_temp.get(0,1);
//                b[0] += lines.get(i1/lines.cols(), i1%lines.cols())[1];
//
//                line_temp.put(0,0, a[0]);
//                line_temp.put(0,1, b[0]);
//            }
//            double[] a2 = line_temp.get(0, 0);
//            double[] b2 = line_temp.get(0,1);
//            a2[0] /= combineIndex.get(i).size();
//            b2[1] /= combineIndex.get(i).size();
//            line_temp.put(0,0, a2[0]);
//            line_temp.put(0,1, b2[0]);
//            combinedLines.push_back(line_temp);
//        }
//        return combinedLines;
//    }

    //typy figurek
    //předpona B pro černé a W pro bílé (diakritika)
    //0 - prázdné pole (empty)
    //1 - pěšák (pawn)
    //2 - král (king)
    //3 - královna (queen)
    //4 - střelec (bishop)
    //5 - kůň (knight)
    //6 - věž (rook)

    /*pohled z boku odkud je foceno - pouze pro účely tvorby datasetu*/
    public static String[] figures = {
            "W6", "W1", "0", "0", "0", "0", "B1", "B6",
            "W5", "W1", "0", "0", "0", "0", "B1", "B5",
            "W4", "W1", "0", "0", "0", "0", "B1", "B4",
            "W3", "W1", "0", "0", "0", "0", "B1", "B2",
            "W2", "W1", "0", "0", "0", "0", "B1", "B3",
            "W4", "W1", "0", "0", "0", "0", "B1", "B4",
            "W5", "W1", "0", "0", "0", "0", "B1", "B5",
            "W6", "W1", "0", "0", "0", "0", "B1", "B6"
    };

    public static final int RECTIFIED_WIDTH = 850;
    public static final int RECTIFIED_HEIGHT = 850;

    public static double average(ArrayList<Double> list){
        Double sum = 0.0;
        if(!list.isEmpty()) {
            for (Double mark : list) {
                sum += mark;
            }
            return sum / list.size();
        }
        return sum;
    }

    public static Mat Resize(Mat m, boolean show){
        Mat mResized = new Mat();
        double ratio = m.width()/(double)m.height();
        Imgproc.resize(m, mResized, new Size(1000*ratio, 1000), 0, 0, Imgproc.INTER_AREA);

        if (show){
            imshow("Resized", mResized);
        }
        return mResized;
    }

    /*---------------------odstraneni sumu-------------------------*/
    private static Mat Denoise(Mat m, boolean show){
        Mat mDenoised = new Mat();
        //Photo.fastNlMeansDenoising(m, mDenoised, 10, 10,7);
        blur(m, mDenoised, new Size(3,3));
        if (show){
            imshow("Denoised", mDenoised);
        }
        return mDenoised;
    }
    /*--------------------odstraneni sumu end---------------------*/

    /*--------------------canny detector--------------------------*/
    private static Mat AutoCannyDetection(Mat m, double sigma,  boolean show){
        Mat mCanny = new Mat();
        Mat mDst = new Mat();

        double highThresh = Imgproc.threshold(m, mDst, 0, 255, THRESH_BINARY + THRESH_OTSU);
        double lowThresh = 0.5*highThresh;
        Canny(m, mCanny, lowThresh, highThresh, 3, false);

        Mat mCannyDilated = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_CROSS, new Size(3, 3));
        dilate(mCanny, mCannyDilated, kernel);

        if (show){
            imshow("Canny Detector", mCanny);
            //imshow("Canny Detector - dilated", mCannyDilated);
        }
        return mCanny;
    }
    /*--------------------canny detector end----------------------*/

    public static ArrayList<Rect> Contours(Mat totalOriginal, Mat m, boolean show){
        Mat original = totalOriginal.clone();
        Mat mContours = m.clone();
        List<MatOfPoint> contours = new ArrayList<>();
        ArrayList<Rect> rectangles = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat mC = new Mat();
        cvtColor(m, mC, COLOR_BGR2GRAY);
        Imgproc.findContours(mC, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat drawing = Mat.zeros(m.size(), CvType.CV_8UC3);
        double areaSum = 0.0;
        int q = 0;
        for (int i = 0; i < contours.size(); i++) {
            Rect r = boundingRect(contours.get(i));
            if (Math.abs(r.width/(double)r.height - 1.0) < 0.3 & r.width > 50 & r.width < 150){ //trideni podle pomeru vysky a sirky - pouze ctverce
                rectangles.add(r);
                areaSum += r.area();
            }
            //Imgproc.drawContours(drawing, contours, i, color, 2, Core.LINE_8, hierarchy, 0, new Point());
        }

        //spocita se prumerna plocha a potom se vyrazne odchylky odstrani
        double averageArea = areaSum/rectangles.size();
        List<Rect> toRemove = new ArrayList<>();
        for (Rect r: rectangles){
            if((r.area()-averageArea) > 2000){
                toRemove.add(r);
            }
        }
        rectangles.removeAll(toRemove);

//        for(Rect rect : rectangles){
//            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
//            rectangle(original, new Point(rect.x, rect.y), new Point(rect.x+rect.width, rect.y+rect.height), color, -1);
//        }


        if(show){
            imshow("Contours", original);
        }
        return rectangles;
    }

    public static void drawSquares(Mat m, ArrayList<Rect> squares, boolean show){
        ArrayList<List<Rect>> rows = createChessboard(squares);
        if (rows == null){
            System.out.println("malo ctvercu rozpoznano");
            return;
        }
        Random rng = new Random(12345);

        for(List<Rect> row: rows){
            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
            for(Rect rect: row){
                rectangle(m, new Point(rect.x, rect.y), new Point(rect.x+rect.width, rect.y+rect.height), color, -1);
            }
        }

        if(show){
            imshow("squares", m);
        }
    }


    /*predpoklada 64 ctvercu - srovna je zleva zespodu po radach*/
    private static ArrayList<List<Rect>> createChessboard(ArrayList<Rect> rectangles) {
        if (rectangles.size() < 64){
            return null;
        }
        //seradi ctverce odspodu nahoru
        rectangles.sort((r1, r2) -> {
            if (r1.y == r2.y) return 0;
            return r1.y > r2.y ? -1 : 1;
        });
        ArrayList<List<Rect>> rows = new ArrayList<>();
        for (int i = 0; i < 8; i++){
            List<Rect> row = rectangles.subList(i*8, i*8+8);
            //seradi ctverce v rade zleva doprava
            row.sort((r1, r2) -> {
               if (r1.x == r2.x) return 0;
               return r1.x < r2.x ? -1 : 1;
            });
            rows.add(row);
        }
        return rows;
    }


    /*------------------houghova transformace--------------------*/
    public static ArrayList<Point> DetectCorners(Mat m, boolean show, int fNum) {
        Mat mHough = m.clone();
        Imgproc.cvtColor(m, mHough, Imgproc.COLOR_GRAY2BGR);

        Mat lines = new Mat(); // will hold the results of the detection
        Imgproc.HoughLines(m, lines, 1, Math.PI/180, 160); // runs the actual detection
        // Draw the lines

        ArrayList<Point> controllPointsH = new ArrayList<>();
        ArrayList<Point> controllPointsV = new ArrayList<>();
        ArrayList<Pair<Point, Point>> horizontalLines = new ArrayList<>();
        ArrayList<Pair<Point, Point>> verticalLines = new ArrayList<>();

        for (int x = 0; x < lines.rows(); x++) {
            double rho = lines.get(x, 0)[0],
                    theta = lines.get(x, 0)[1];
            double a = Math.cos(theta), b = Math.sin(theta);
            double x0 = a * rho, y0 = b * rho;
            Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
            Point pt2 = new Point(Math.round(x0 - 1500 * (-b)), Math.round(y0 - 1500 * (a)));
           // Imgproc.line(mHough, pt1, pt2, new Scalar(0,0,255), 1, Imgproc.LINE_AA, 0);




            //rozdeleni podle smeru cary
            if (Math.abs(pt1.y - pt2.y) < 300){ //horizontalni
                Point controll = new Point(Math.round(x0 - 700*(-b)), Math.round(y0 - 700*(a)));

                boolean duplicate = false;
                for (Point c: controllPointsH){
                    if ((Math.abs(c.y - controll.y) < 30) & (Math.abs(c.x - controll.x) < 30)){
                        duplicate = true;
                    }
                }

                if (!duplicate) {
                    controllPointsH.add(controll);
                    horizontalLines.add(new Pair<>(pt1, pt2));
                    Imgproc.line(mHough, pt1, pt2, new Scalar(255,192,203), 2, Imgproc.LINE_AA, 0);
                } else {
                }
            } else if (Math.abs(pt1.x - pt2.x) < 1200) { //vertikalni
                Point controll = new Point(Math.round(x0 - 700*(-b)), Math.round(y0 - 700*(a)));

                boolean duplicate = false;
                for (Point c: controllPointsV){
                    if ((Math.abs(c.y - controll.y) < 30) & (Math.abs(c.x - controll.x) < 30)){
                        duplicate = true;
                    }
                }

                if (!duplicate) {
                    controllPointsV.add(controll);
                    verticalLines.add(new Pair<>(pt1, pt2));
                    Imgproc.line(mHough, pt1, pt2, new Scalar(0, 0, 255), 2, Imgproc.LINE_AA, 0);
                } else {
                }
            }
        }

            /*------TODO TOHLE TADY NEBUDE - rozpoznani rohu sachovnice----*/
        ArrayList<Point> corners = new ArrayList<>();

        /*razeni car*/
        horizontalLines.sort((l1,l2) -> {
            if (l1.getKey().y == l2.getKey().y) return 0;
            return l1.getKey().y > l2.getKey().y ? 1 : -1;
        });

        verticalLines.sort((l1,l2) -> { //musi se vybrat vzdy jen spodni bod
            Point p1 = l1.getKey();
            if (p1.y < 0){
                p1 = l1.getValue();
            }
            Point p2 = l2.getKey();
            if (p2.y < 0){
                p2 = l2.getValue();
            }
            if (p1.x == p2.x) return 0;
            return p1.x > p2.x ? 1 : -1;
        });

        /*VYBER 9 SPRAVNYCH CAR*/
        ArrayList<Double> distArray = new ArrayList<>();
        for (int i=1; i<verticalLines.size()-2; i++){ //vetsi sance na spravny prumer kdyz vynecham kraje
            Pair<Point, Point> line1 = verticalLines.get(i);
            Pair<Point, Point> line2 = verticalLines.get(i+1);
            Point p1 = line1.getKey();
            if (p1.y < 0){
                p1 = line1.getValue();
            }
            Point p2 = line2.getKey();
            if (p2.y < 0){
                p2 = line2.getValue();
            }
            double dist = p2.x - p1.x;
            System.out.println(dist);
            distArray.add(dist);
        }
        distArray = new ArrayList<Double>(distArray.subList(distArray.size()/2-2, distArray.size()/2+2));
        double avgDist = average(distArray);
        System.out.println(avgDist);

        double minDist = 10000;
        int index = 0;
        int minIndex = 0;
        System.out.println("-------POINTS-------");
        System.out.println("WIDTH = " + m.width());
        for(Pair<Point, Point> line: verticalLines){

            Point p = line.getKey().y > line.getValue().y ? line.getKey() : line.getValue();
            if (Math.abs(p.x-m.width()/2.0) < minDist){
                minDist = Math.abs(p.x-m.width()/2.0);
                minIndex = index;
            }
            System.out.println(p.x);
            index++;
        }
        System.out.println("nejbliz je index " + minIndex + " a dist je " + minDist);

//        for(int i=0; i<verticalLines.size()-1; i++){
//            Pair<Point, Point> line1 = verticalLines.get(i);
//            Pair<Point, Point> line2 = verticalLines.get(i+1);
//            Point p1 = line1.getKey();
//            if (p1.y < 0){
//                p1 = line1.getValue();
//            }
//            Point p2 = line2.getKey();
//            if (p2.y < 0){
//                p2 = line2.getValue();
//            }
//            double dist = p2.x - p1.x;
//            if (Math.abs(dist-avgDist) < 30){
//                Imgproc.line(mHough, line1.getKey(), line1.getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
//                Imgproc.line(mHough, line2.getKey(), line2.getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
//            } else {
//                if (i>verticalLines.size()/2){
//                    Imgproc.line(mHough, line2.getKey(), line2.getValue(), new Scalar(255, 255, 255), 2, Imgproc.LINE_AA, 0);
//                } else {
//                    Imgproc.line(mHough, line1.getKey(), line1.getValue(), new Scalar(255, 255, 255), 2, Imgproc.LINE_AA, 0);
//                }
//            }
//        }



        /*VYBER 9 SPRAVNYCH CAR - HORIZONTALNE*/
        double minDistH = 10000;
        int indexH = 0;
        int minIndexH = 0;
        System.out.println("-------POINTS-------");
        System.out.println("HEIGHT = " + m.height());
        for(Pair<Point, Point> line: verticalLines){

            Point p = line.getKey();
            if (Math.abs(p.x-m.height()/2.0) < minDistH){
                minDistH = Math.abs(p.x-m.height()/2.0);
                minIndexH = indexH;
            }
            System.out.println(p.x);
            indexH++;
        }
        System.out.println("nejbliz je indexH " + minIndexH + " a distH je " + minDistH);

        Pair<Point, Point> lineH = horizontalLines.get(minIndexH);
        Pair<Point, Point> lineV = verticalLines.get(minIndex);
        Point center = intersection(
                lineH.getKey().x,
                lineH.getKey().y,
                lineH.getValue().x,
                lineH.getValue().y,
                lineV.getKey().x,
                lineV.getKey().y,
                lineV.getValue().x,
                lineV.getValue().y
        );

        Imgproc.circle(mHough, center, 20, new Scalar(232, 204, 215), -1);

        //Imgproc.line(mHough, horizontalLines.get(0).getKey(), horizontalLines.get(0).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
        //Imgproc.line(mHough, horizontalLines.get(horizontalLines.size()-1).getKey(), horizontalLines.get(horizontalLines.size()-1).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
        //Imgproc.line(mHough, verticalLines.get(0).getKey(), verticalLines.get(0).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
        //Imgproc.line(mHough, verticalLines.get(verticalLines.size()-1).getKey(), verticalLines.get(verticalLines.size()-1).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);

        corners.add(intersection(
                    horizontalLines.get(0).getKey().x,
                    horizontalLines.get(0).getKey().y,
                    horizontalLines.get(0).getValue().x,
                    horizontalLines.get(0).getValue().y,
                    verticalLines.get(0).getKey().x,
                    verticalLines.get(0).getKey().y,
                    verticalLines.get(0).getValue().x,
                    verticalLines.get(0).getValue().y));

        corners.add(intersection(
                    horizontalLines.get(0).getKey().x,
                    horizontalLines.get(0).getKey().y,
                    horizontalLines.get(0).getValue().x,
                    horizontalLines.get(0).getValue().y,
                    verticalLines.get(verticalLines.size()-1).getKey().x,
                    verticalLines.get(verticalLines.size()-1).getKey().y,
                    verticalLines.get(verticalLines.size()-1).getValue().x,
                    verticalLines.get(verticalLines.size()-1).getValue().y));

        corners.add(intersection(
                    horizontalLines.get(horizontalLines.size()-1).getKey().x,
                    horizontalLines.get(horizontalLines.size()-1).getKey().y,
                    horizontalLines.get(horizontalLines.size()-1).getValue().x,
                    horizontalLines.get(horizontalLines.size()-1).getValue().y,
                    verticalLines.get(0).getKey().x,
                    verticalLines.get(0).getKey().y,
                    verticalLines.get(0).getValue().x,
                    verticalLines.get(0).getValue().y));

        corners.add(intersection(
                    horizontalLines.get(horizontalLines.size()-1).getKey().x,
                    horizontalLines.get(horizontalLines.size()-1).getKey().y,
                    horizontalLines.get(horizontalLines.size()-1).getValue().x,
                    horizontalLines.get(horizontalLines.size()-1).getValue().y,
                    verticalLines.get(verticalLines.size()-1).getKey().x,
                    verticalLines.get(verticalLines.size()-1).getKey().y,
                    verticalLines.get(verticalLines.size()-1).getValue().x,
                    verticalLines.get(verticalLines.size()-1).getValue().y));

        Imgproc.circle(mHough, corners.get(0), 10, new Scalar(0,255,0), -1);
        Imgproc.circle(mHough, corners.get(1), 10, new Scalar(0,255,0), -1);
        Imgproc.circle(mHough, corners.get(2), 10, new Scalar(0,255,0), -1);
        Imgproc.circle(mHough, corners.get(3), 10, new Scalar(0,255,0), -1);

        if (show) {
            imshow("Hough Transform"+fNum, mHough);
            waitKey();
        }

        return corners;
    }

    public static Point intersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
        if (d == 0) return null;

        double xi = ((x3-x4)*(x1*y2-y1*x2)-(x1-x2)*(x3*y4-y3*x4))/d;
        double yi = ((y3-y4)*(x1*y2-y1*x2)-(y1-y2)*(x3*y4-y3*x4))/d;

        return new Point(xi,yi);
    }

    /*----------------houghova transformace end-------------------*/

/*------------------------rektifikace-------------------------*/
    public static Mat Rectification(Mat m, ArrayList<Point> corners, boolean show, int fNum){
        Point [] srcArray = new Point[4];
        srcArray[0] = new Point(corners.get(0).x, corners.get(0).y);
        srcArray[1] = new Point(corners.get(1).x, corners.get(1).y);
        srcArray[2] = new Point(corners.get(2).x, corners.get(2).y);
        srcArray[3] = new Point(corners.get(3).x, corners.get(3).y);

        LinkedList<Point> dstArray = new LinkedList<>();

        int x = 100;
        int y = 100;

        dstArray.add(new Point(x,y));
        dstArray.add(new Point(x + RECTIFIED_WIDTH, y));
        dstArray.add(new Point(x,y + RECTIFIED_HEIGHT));
        dstArray.add(new Point(x + RECTIFIED_WIDTH, y + RECTIFIED_HEIGHT));

        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstArray);

        MatOfPoint2f src = new MatOfPoint2f();
        src.fromArray(srcArray);

        Mat mRectified = new Mat();
        Mat homography = findHomography(src, dst);
        warpPerspective(m, mRectified, homography, new Size(m.cols(), m.rows()));
        //TODO udělat obrázek menší (podle velikosti šachovnice)
        if(show){
            imshow("Rectified_"+fNum, mRectified);
            waitKey();
            imwrite("D:/School/2MIT/DP/teeeeeeeeeeeeeeeeeeest.jpg", mRectified);
        }
        return mRectified;
    }
/*-----------------------rektifikace end-----------------------*/

    /**
     *
     * @param totalOriginal originalni obrazek
     * @param rectangleContours pozice ctvercu, ktere ma rezat
     * @return arraylist matic ctvercu
     */
    private static ArrayList<Mat> cutSquares(Mat totalOriginal, ArrayList<Rect> rectangleContours) {
        ArrayList<List<Rect>> rows = createChessboard(rectangleContours);
        ArrayList<Mat> result = new ArrayList<>();
        assert rows != null;
        int i = 0;
        for (List<Rect> row : rows) {
            for(Rect square : row){
                i++;
                Mat squareCut = totalOriginal.submat(square);
                result.add(squareCut);

//                List<Mat> images = new ArrayList<>();
//                images.add(squareCut);
//                Mat hist = new Mat();
//                int histSize = 256;
//                float[] range = {0, 256}; //the upper boundary is exclusive
//                MatOfFloat histRange = new MatOfFloat(range);
//                calcHist(images, new MatOfInt(0), new Mat(), hist, new MatOfInt(histSize), histRange, false);
//                imwrite("D:/School/2MIT/DP/results/test_3/"+i+".jpg", squareCut);
            }
        }
        return result;
    }

    /*predpoklada REKTIFIKOVANY OBRAZEK*/
    public static void cutSquares2(Mat m, ArrayList<Point> corners){
        ArrayList<Rect> squares = new ArrayList<>();

//        Point[] cornersSorted = new Point[4];
//        for (Point corner: corners){ //serazeni rohu - VLEVO NAHORE JE PRVNI A PAK PO SMERU HOD. RUCICEK
//            if (corner.x < m.width()/2.0){
//                if (corner.y < m.height()/2.0){
//                    cornersSorted[0] = corner;
//                } else {
//                    cornersSorted[2] = corner;
//                }
//            } else {
//                if (corner.y < m.height()/2.0){
//                    cornersSorted[1] = corner;
//                } else {
//                    cornersSorted[3] = corner;
//                }
//            }
//        }

        int squareSize = RECTIFIED_WIDTH/8;
        double[] heightQuantifiers = {2,2.5,2.5,3,3,3,3.5,3.5};
        double[] widthQuantifiers = {1.2,1.3,1.4,1.5,1.6,1.6,1.8,2};
        for (int i = 0; i<8; i++){
            for (int j = 0; j<8; j++){
                //Rect r = new Rect(100+j*squareSize, 100+i*squareSize, (int) (squareSize*widthQuantifiers[7-i]), (int) (squareSize*heightQuantifiers[7-i]));
                Rect r = new Rect(100+j*squareSize, 100+i*squareSize, squareSize, squareSize);
                squares.add(r);
            }
        }

        int i = 0;
        for (Rect square: squares){
            Mat s = m.submat(square);
            imwrite("D:/School/2MIT/DP/data/test/" + i +".jpg", s);
            i++;
        }
//        int i = 0;
//        for (Rect square: squares){
//            Mat s = m.submat(square);
//            int index = new File("D:/School/2MIT/DP/data/" + figures[i]).list().length;
//            imwrite("D:/School/2MIT/DP/data/" + figures[i] + index +".jpg", s);
//            i++;
//        }

    }

    private static Mat preRectification(Mat m, ArrayList<Point> corners, boolean show, int fNum) {
        Point [] srcArray = new Point[4];
        srcArray[0] = new Point(corners.get(0).x, corners.get(0).y);
        srcArray[1] = new Point(corners.get(1).x, corners.get(1).y);
        srcArray[2] = new Point(corners.get(2).x, corners.get(2).y);
        srcArray[3] = new Point(corners.get(3).x, corners.get(3).y);

        double upperWidth = srcArray[1].x - srcArray[0].x;
        double bottomWidth = srcArray[3].x - srcArray[2].x;
        System.out.println("upper= "+upperWidth+" bottom= "+bottomWidth);

        LinkedList<Point> dstArray = new LinkedList<>();

        double avgLeft = (srcArray[0].x+srcArray[2].x)/2;
        double avgRight = (srcArray[1].x+srcArray[3].x)/2;
        double height = avgRight - avgLeft;


        dstArray.add(new Point(avgLeft,srcArray[2].y-800));
        dstArray.add(new Point(avgLeft+800, srcArray[3].y-800));
        dstArray.add(new Point(avgLeft, srcArray[2].y));
        dstArray.add(new Point(avgLeft+800, srcArray[3].y));


        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstArray);

        MatOfPoint2f src = new MatOfPoint2f();
        src.fromArray(srcArray);

        Mat mRectified = new Mat();
        Mat homography = findHomography(src, dst);

        warpPerspective(m, mRectified, homography, new Size(avgRight+50, srcArray[2].y+50));
        //TODO udělat obrázek menší (podle velikosti šachovnice)
        if(show){
            imshow("PREEEERectified_"+fNum, mRectified);
            waitKey();
        }
        return mRectified;
    }

    public static void processImage(Mat frame, int i){
        Mat totalOriginal = new Mat();
        totalOriginal = frame;
        //pouze pri videu!
        //cvtColor(totalOriginal, totalOriginal, COLOR_BGR2GRAY);

        totalOriginal = Resize(totalOriginal, false);

        totalOriginal = Denoise(totalOriginal, false);

        Mat original = totalOriginal.clone();

        original = AutoCannyDetection(original, 0.33, false);

        //FUNGUJE KDYŽ PŘIJDE OPRAVDU 9X9 čar
        ArrayList<Point> corners;
        corners = DetectCorners(original, true , i);

        //FUNGUJE KDYŽ PŘIJDOU 4 ROHY
        Rectification(totalOriginal, corners, true, i);
        //original = preRectification(totalOriginal, corners, true, i);
    }

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        BoardRecognizer recognizer = new BoardRecognizer();
        //Mat totalOriginal = imread("D:/School/2MIT/DP/template.jpg", Imgcodecs.IMREAD_GRAYSCALE);
        //recognizer.processImage(totalOriginal);

        //VideoCapture cap = new VideoCapture("rtsp://192.168.0.100:8080/h264_pcm.sdp");
//        VideoCapture cap = new VideoCapture("D:/School/2MIT/DP/video4.mp4");
//        Mat frame = new Mat();
//        boolean ret;
//        int i = 0;
//        while(cap.isOpened()){
//            //System.out.println("jedem");
//            ret = cap.read(frame);
//            if (ret){
//                i++;
//                if (i%60==0){
//                    //imshow("frame_"+i, frame);
//                    //waitKey();
//                    recognizer.processFrame(frame, i);
//                    //imshow("frame"+i,frame);
//                    //waitKey();
//                    //imwrite("videoFrames/video_"+i+".jpg", frame);
//                }
//
//            } else {
//                System.out.println("aa");
//                break;
//            }
//        }
//        cap.release();

        for(int i = 1; i<25; i++){
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/X"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE);
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v2/X"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE);
            Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v3/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE);

            //Mat totalOriginal = imread("D:/School/2MIT/DP/cislovani/2.jpg", Imgcodecs.IMREAD_GRAYSCALE);
            String dir = System.getProperty("user.dir");
            System.out.println(dir);
            //Mat totalOriginal = imread("D:/School/2MIT/DP/side_3.jpg", IMREAD_GRAYSCALE);
            //Mat totalOriginal = imread(dir+"/src/main/resources/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE);
            Mat toSave = totalOriginal.clone();
            recognizer.processImage(totalOriginal);
            //imwrite("D:/School/2MIT/DP/picsForDataset/X"+i+".jpg", toSave);
            //imwrite("D:/School/2MIT/DP/picsForDataset/v2/X"+i+".jpg", toSave);
        }


        //Mat totalOriginal = imread("D:/School/2MIT/DP/test_99.jpg", Imgcodecs.IMREAD_GRAYSCALE);
        //processImage(totalOriginal, 1);
//        totalOriginal = Resize(totalOriginal, true);
//
//        totalOriginal = Denoise(totalOriginal, false);
//
//        Mat original = totalOriginal.clone();
//
//        totalOriginal = preRectification(totalOriginal);

//        original = AutoCannyDetection(original, 0.33, false);
//
//        //FUNGUJE KDYŽ PŘIJDE OPRAVDU 9X9 čar
//        ArrayList<Point> corners;
//        corners = DetectCorners(original, true);
//
//        //FUNGUJE KDYŽ PŘIJDOU 4 ROHY
//        original = Rectification(totalOriginal, corners, true);

        //cutSquares2(original, corners);

//        ArrayList<Rect> rectangleContours = Contours(totalOriginal, original, false);
//        drawSquares(original, rectangleContours, true);
//        ArrayList<Mat> squares = cutSquares(totalOriginal, rectangleContours);
//        ArrayList<Mat> histograms = createHistogramList(squares);
//        for (int i = 0; i<64; i++) {
//            histogramToFile(histograms.get(i), i, squares.get(i));
//        }
//        sortSquaresByColor(histograms, squares);
//        imshow("total original", totalOriginal);

         // waitKey();
    }
}
