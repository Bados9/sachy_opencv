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
import static org.opencv.imgcodecs.Imgcodecs.imread;
import static org.opencv.imgcodecs.Imgcodecs.imwrite;
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


//        double rho_threshold= 15;
//        double theta_threshold = Math.toRadians(3);
//        Mat linesC = contourCluster(lines, rho_threshold, theta_threshold);

//        ArrayList<ArrayList<Pair<Point, Point>>> linesC= new ArrayList<>(1);
//        linesC.add (new ArrayList<>());
//
//        double r = lines.get(0,0)[0],
//                t = lines.get(0, 0)[1];
//        double aa = Math.cos(t), bb = Math.sin(t);
//        double xx0 = aa * r, yy0 = bb * r;
//        Point p1 = new Point(Math.round(xx0 + 1000 * (-bb)), Math.round(yy0 + 1000 * (aa)));
//        Point p2 = new Point(Math.round(xx0 - 1500 * (-bb)), Math.round(yy0 - 1500 * (aa)));
//        System.out.println("prvni cara ma bod1 = " + p1 + " a bod2 = " + p2);
//        linesC.get(0).add(new Pair<>(p1, p2));
//
//        int clusters = 1;
//
//        double deltaX = 100;
//        double deltaY = 100;
//
//        for (int i = 1; i < lines.rows(); i++) {
//            double rho = lines.get(i, 0)[0],
//                    theta = lines.get(i, 0)[1];
//            double a = Math.cos(theta), b = Math.sin(theta);
//            double x0 = a * rho, y0 = b * rho;
//            Point pt11 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
//            Point pt12 = new Point(Math.round(x0 - 1500 * (-b)), Math.round(y0 - 1500 * (a)));
//
//            System.out.println("NOVA CARA - bod1 = " + pt11 + " bod2 = " + pt12);
//            Point pt21 = null;
//            Point pt22 = null;
//            for (int j = 0; j < linesC.size(); j++) {
//                pt21 = linesC.get(j).get(0).getKey();
//                pt22 = linesC.get(j).get(0).getValue();
//                if (Math.abs(pt11.x-pt12.x) < 200) {
//                    if ((Math.abs(pt11.x - pt21.x) < deltaX) & (Math.abs(pt12.x - pt22.x) < deltaX)) {
//                        linesC.get(j).add(new Pair<>(pt11, pt12));
//                        System.out.println("A-Pridano do clusteru " + j + ": bod1 = " + pt11 + " bod2 = " + pt12);
//                        break;
//                    }
//                } else {
//                    if ((Math.abs(pt11.y - pt21.y) < deltaY) & (Math.abs(pt12.y - pt22.y) < deltaY)) {
//                        linesC.get(j).add(new Pair<>(pt11, pt12));
//                        System.out.println("B-Pridano do clusteru" + j + ": bod1 = " + pt11 + " bod2 = " + pt12);
//                        break;
//                    }
//                }
//            }
//            linesC.add(new ArrayList<>());
//            linesC.get(clusters).add(new Pair<>(pt11, pt12));
//            clusters += 1;
//        }
//
//        Random rng = new Random(12345);
//        for (ArrayList<Pair<Point, Point>> cluster: linesC) {
//            System.out.println(cluster);
//        }
//
//        for(int y = 0; y < linesC.size(); y++){
//            ArrayList<Pair<Point, Point>> cluster = linesC.get(y);
//            double sumX1 = 0, sumY1 = 0, sumX2 = 0, sumY2 = 0;
//            for(Pair<Point, Point> line: cluster){
//                sumX1 += line.getKey().x;
//                sumY1 += line.getKey().y;
//                sumX2 += line.getValue().x;
//                sumY2 += line.getValue().y;
//
//            }
//            double avgX1 = sumX1 / cluster.size();
//            double avgY1 = sumY1 / cluster.size();
//            double avgX2 = sumX2 / cluster.size();
//            double avgY2 = sumY2 / cluster.size();
//
//            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
//
//            Imgproc.line(mHough, new Point(avgX1, avgY1), new Point(avgX2, avgY2), color, 1, Imgproc.LINE_AA, 0);
//
//        }





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



        //vybira se tech 8 spravnych
        /*double prevDist = 0;
        for(int i = 0; i < horizontalLines.size()-1; i++){
            if (i == 8){
                break;
            }
            Pair<Point, Point> line1 = horizontalLines.get(i);
            Pair<Point, Point> line2 = horizontalLines.get(i+1);
            double dist = Math.abs(line1.getKey().y - line2.getKey().y);
            //System.out.println("i = " + i + "   " + dist);

            if (prevDist == 0 & dist > 70.0){
                horizontalLines.remove(horizontalLines.get(i));
                i--;
            } else if (dist > prevDist){
                prevDist = dist;
            } else {
                horizontalLines.remove(horizontalLines.get(i+1));
                i--;
            }
        }*/

        Imgproc.line(mHough, horizontalLines.get(0).getKey(), horizontalLines.get(0).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
        Imgproc.line(mHough, horizontalLines.get(horizontalLines.size()-1).getKey(), horizontalLines.get(horizontalLines.size()-1).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
        Imgproc.line(mHough, verticalLines.get(0).getKey(), verticalLines.get(0).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);
        Imgproc.line(mHough, verticalLines.get(verticalLines.size()-1).getKey(), verticalLines.get(verticalLines.size()-1).getValue(), new Scalar(0, 255, 0), 2, Imgproc.LINE_AA, 0);

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
        }
        return mRectified;
    }
/*-----------------------rektifikace end-----------------------*/

    private static void histogramToFile(Mat hist, int k, Mat squareCut){
        int histSize = 256;
        int histW = 512, histH = 400;
        int binW = (int) Math.round((double) histW / histSize);
        Mat histImage = new Mat( histH, histW, CvType.CV_8UC3, new Scalar( 0,0,0) );
        Core.normalize(hist, hist, 0, histImage.rows(), Core.NORM_MINMAX);

        //TODO porovnat histogram s ukázkovými
        Mat blackHist = new Mat();
        Mat whiteHist = new Mat();

        float[] range = {0, 256}; //the upper boundary is exclusive
        MatOfFloat histRange = new MatOfFloat(range);
        Mat blackSquare = imread("D:/School/2MIT/DP/black_square.jpg");
        Mat whiteSquare = imread("D:/School/2MIT/DP/white_square.jpg");
        List<Mat> blackList= new ArrayList<>();
        blackList.add(blackSquare);
        List<Mat> whiteList= new ArrayList<>();
        whiteList.add(blackSquare);
        calcHist(blackList, new MatOfInt(0), new Mat(), blackHist, new MatOfInt(histSize), histRange, false);
        calcHist(whiteList, new MatOfInt(0), new Mat(), whiteHist, new MatOfInt(histSize), histRange, false);

        double isBlack = compareHist(blackHist, hist, CV_COMP_INTERSECT);
        double isWhite = compareHist(whiteHist, hist, CV_COMP_INTERSECT);

        System.out.println("cislo " + k + " ma vysledek " + isBlack);
        float[] histData = new float[(int) (hist.total() * hist.channels())];
        hist.get(0, 0, histData);


        for( int i = 1; i < histSize; i++ ) {
            Imgproc.line(histImage, new Point(binW * (i - 1), histH - Math.round(histData[i - 1])),
                    new Point(binW * (i), histH - Math.round(histData[i])), new Scalar(255, 255, 255), 2);
        }

        imwrite("D:/School/2MIT/DP/results/test_3/hist/"+k+"_hist_.jpg", histImage);
        imwrite("D:/School/2MIT/DP/results/test_3/hist/"+k+".jpg", squareCut);
    }

    /**
     * vypocita histogramy pro vsechny ctverce
     * @param squares seznam ctvercu z funkce cutSquares()
     * @return arraylist histogramu techto ctvercu
     */
    private static ArrayList<Mat> createHistogramList(ArrayList<Mat> squares){
        ArrayList<Mat> result = new ArrayList<>();

        for (Mat square : squares){
            List<Mat> list = new ArrayList<>();
            list.add(square);
            Mat hist = new Mat();
            int histSize = 256;
            float[] range = {0, 256}; //the upper boundary is exclusive
            MatOfFloat histRange = new MatOfFloat(range);
            calcHist(list, new MatOfInt(0), new Mat(), hist, new MatOfInt(histSize), histRange, false);
            result.add(hist);
        }

        return result;
    }

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
        //Rectification(totalOriginal, corners, true, i);
        preRectification(totalOriginal, corners, true, i);
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


        dstArray.add(new Point(avgLeft,srcArray[2].y-height));
        dstArray.add(new Point(avgRight, srcArray[3].y-height));
        dstArray.add(new Point(avgLeft, srcArray[2].y));
        dstArray.add(new Point(avgRight, srcArray[3].y));


        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstArray);

        MatOfPoint2f src = new MatOfPoint2f();
        src.fromArray(srcArray);

        Mat mRectified = new Mat();
        Mat homography = findHomography(src, dst);
        warpPerspective(m, mRectified, homography, new Size(m.cols(), m.rows()));
        //TODO udělat obrázek menší (podle velikosti šachovnice)
        if(show){
            imshow("PREEEERectified_"+fNum, mRectified);
            waitKey();
        }
        return mRectified;
    }

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

//        VideoCapture cap = new VideoCapture("rtsp://192.168.0.100:8080/h264_pcm.sdp");
//        Mat frame = new Mat();
//        boolean ret;
//        int i = 0;
//        while(cap.isOpened()){
//            //System.out.println("jedem");
//            ret = cap.read(frame);
//            if (ret){
//                i++;
//                if (i%60==0){
//                    processImage(frame);
//                    //imshow("frame"+i,frame);
//                    waitKey();
//                    imwrite("videoFrames/video_"+i+".jpg", frame);
//                }
//
//            } else {
//                System.out.println("aa");
//                break;
//            }
//        }
//        cap.release();


        for(int i = 1; i<22; i++){
            Mat totalOriginal = imread("D:/School/2MIT/DP/test_"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE);
            processImage(totalOriginal, i);
        }

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

          waitKey();
    }
}
