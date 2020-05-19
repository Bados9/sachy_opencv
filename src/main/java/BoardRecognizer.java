import javafx.util.Pair;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.*;

import static java.lang.Math.*;
import static org.opencv.calib3d.Calib3d.findHomography;
import static org.opencv.highgui.HighGui.imshow;
import static org.opencv.highgui.HighGui.waitKey;
import static org.opencv.imgcodecs.Imgcodecs.imwrite;
import static org.opencv.imgproc.Imgproc.*;

class BoardLines{
    public ArrayList<Pair<Point, Point>> verticalLines;
    public ArrayList<Pair<Point, Point>> horizontalLines;
    public Mat frame;

    public BoardLines(ArrayList<Pair<Point, Point>> verticalLines, ArrayList<Pair<Point, Point>> horizontalLines, Mat frame){
        this.verticalLines = verticalLines;
        this.horizontalLines = horizontalLines;
        this.frame = frame;
    }
}

public class BoardRecognizer {
    public Mat original;
    public Mat rectified;
    public int iNum;

    public BoardRecognizer(){

    }
    //cerne-leva klavesnice
    //bile-prava klavesnice
    //0 - prázdné pole (empty)
    //1 - pěšák (pawn)
    //2 - král (king)
    //3 - královna (queen)
    //4 - střelec (bishop)
    //5 - kůň (knight)
    //6 - věž (rook)
    public void show(String text, Mat image, boolean save){
        imshow(iNum+"_"+text, image);
        iNum++;
        if (save){
            int key = waitKey(0);
            int index;
            switch(key){
                case 98:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/white_king").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/white_king/white_king." + index + ".jpg", image);
                    break;
                case 50:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/black_king").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/black_king/black_king." + index + ".jpg", image);
                    break;
                case 99:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/white_queen").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/white_queen/white_queen." + index + ".jpg", image);
                    break;
                case 51:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/black_queen").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/black_queen/black_queen." + index + ".jpg", image);
                    break;
                case 97:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/white_pawn").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/white_pawn/white_pawn." + index + ".jpg", image);
                    break;
                case 49:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/black_pawn").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/black_pawn/black_pawn." + index + ".jpg", image);
                    break;
                case 101:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/white_knight").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/white_knight/white_knight." + index + ".jpg", image);
                    break;
                case 53:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/black_knight").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/black_knight/black_knight." + index + ".jpg", image);
                    break;
                case 100:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/white_bishop").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/white_bishop/white_bishop." + index + ".jpg", image);
                    break;
                case 52:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/black_bishop").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/black_bishop/black_bishop." + index + ".jpg", image);
                    break;
                case 102:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/white_rook").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/white_rook/white_rook." + index + ".jpg", image);
                    break;
                case 54:
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/black_rook").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/black_rook/black_rook." + index + ".jpg", image);
                    break;
                case 10: //enter
                case 32: //mezernik
                    index = Objects.requireNonNull(new File("D:/School/2MIT/DP/data_one_empty/empty").list()).length;
                    index += 100;
                    imwrite("D:/School/2MIT/DP/data_one_empty/empty/empty." + index + ".jpg", image);
                    break;
                default : //neukladat
                    break;
            }
        } else {
            waitKey();
        }

    }

    public Mat resize(Mat m){
        Mat mResized = new Mat();
        double ratio = m.width()/(double)m.height();
        Imgproc.resize(m, mResized, new Size(1000*ratio, 1000), 0, 0, Imgproc.INTER_AREA);

        return mResized;
    }

    public Mat preRectification(Mat m){
        Point [] srcArray = new Point[4];
        srcArray[0] = new Point(180,880);
        srcArray[1] = new Point(m.width()-180,880);
        srcArray[2] = new Point(330,150);
        srcArray[3] = new Point(m.width()-330,150);

        LinkedList<Point> dstArray = new LinkedList<>();

        dstArray.add(new Point(100,850));
        dstArray.add(new Point(900,850));
        dstArray.add(new Point(100,100));
        dstArray.add(new Point(900,100));

        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstArray);

        MatOfPoint2f src = new MatOfPoint2f();
        src.fromArray(srcArray);

        Mat rectified = new Mat();
        Mat homography = findHomography(src, dst);
        warpPerspective(m, rectified, homography, new Size(m.cols(), m.rows()));

        Rect crop = new Rect(new Point(100, 50), new Point(900,850));
        rectified = rectified.submat(crop);

        return rectified;
    }

    public void processImage(Mat image){
        image = resize(image);
        original = image;

        show("original",original, false);
        preProcess(image);
    }

    private Mat preProcess(Mat image) {
        Mat laplace = new Mat();

        image = preRectification(image);

        rectified = image.clone();

        Mat imageForCutting = image.clone();

        Laplacian(image, laplace, 0);

        show("laplace", laplace, false);

        BoardLines allLines = houghTest(laplace);

        ArrayList<Rect> squares = createSquares(allLines);

        if (squares == null){
            System.out.println("malo ctvercu");
            return null;
        }

        ArrayList<Rect> sortedSquares = sortSquares(squares);

        cutSquares2(imageForCutting, sortedSquares);

        return laplace;
    }

    private ArrayList<Rect> sortSquares(ArrayList<Rect> squares) {
        System.out.println(squares.size());
        squares.sort((s1,s2) -> {
            if (s1.y == s2.y) return 0;
            return s1.y > s2.y ? -1 : 1;
        });

        ArrayList<Rect> sortedSquares = new ArrayList<>();

        for (int i = 0; i<8; i++){
            List<Rect> temp = squares.subList(8*i,8*i+8);
            temp.sort((s1,s2)-> {
               if (s1.x == s2.x) return 0;
               return s1.x > s2.x ? -1 : 1;
            });
            sortedSquares.addAll(temp);
        }
        return sortedSquares;
    }

    private ArrayList<Mat> cutSquares2(Mat totalOriginal, ArrayList<Rect> rectangleContours) {
        ArrayList<Mat> result = new ArrayList<>();
        int i = 0;
        for(Rect square: rectangleContours){
            i++;
            square.x-=10;
            square.width+=20;
            square.height+=10;

            if (i>32){
                square.y-=square.height*0.8;
                square.height*=1.8;
            } else {
                square.y-=square.height/2;
                square.height*=1.5;
            }

            square.x = max(square.x, 0);
            square.y = max(square.y, 0);
            square.height = min(totalOriginal.height()-square.y, square.height);
            square.width = min(totalOriginal.width()-square.x, square.width);

            Mat squareCut = totalOriginal.submat(square);
            result.add(squareCut);
            show("square_"+i, squareCut, true);
        }
        return result;
    }

    public ArrayList<Rect> createSquares(BoardLines lines){
        List<MatOfPoint> contours = new ArrayList<>();
        ArrayList<Rect> rectangles = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(lines.frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            Rect r = boundingRect(contour);
            if (r.width > 200 || r.height > 200) {
                continue;
            }
            rectangles.add(r);
        }

        Random rng = new Random(12345);
        for(Rect rect : rectangles){
            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
            rectangle(rectified, new Point(rect.x, rect.y), new Point(rect.x+rect.width, rect.y+rect.height), color, -1);
        }

        show("rectangles new", rectified, false);

        if(rectangles.size() == 64){
            return rectangles;
        } else {
            return null; //todo asi pres vyjimky
        }

    }

    public BoardLines houghTest(Mat image){
        Mat dst = image.clone(); Mat cdst = new Mat();

        Imgproc.Canny(dst, dst, 50, 200, 3, false);
        Imgproc.cvtColor(dst, cdst, Imgproc.COLOR_GRAY2BGR);

        Mat linesP = new Mat();
        Imgproc.HoughLinesP(dst, linesP, 0.85, Math.PI/280, 190, 150, 100); // runs the actual detection

        ArrayList<Pair<Point, Point>> verticalLines = new ArrayList<>();
        ArrayList<Pair<Point, Point>> horizontalLines = new ArrayList<>();

        for (int x = 0; x < linesP.rows(); x++) {
            double[] l = linesP.get(x, 0);
            double tx = l[2] - l[0];
            double ty = l[3] - l[1];
            double vecLength = sqrt(pow(tx,2)+pow(ty,2));
            tx /= vecLength;
            ty /= vecLength;
            if (abs(abs(tx)-abs(ty))<0.5){

            } else if (abs(tx) > abs(ty)) {
                horizontalLines.add(new Pair<>(new Point(l[0],l[1]), new Point(l[2],l[3])));
            } else {
                verticalLines.add(new Pair<>(new Point(l[0],l[1]), new Point(l[2],l[3])));
            }
        }

        horizontalLines = removeDuplicates(horizontalLines, 0);
        verticalLines = removeDuplicates(verticalLines, 1);

        horizontalLines = correctLines(horizontalLines, 0);
        verticalLines = correctLines(verticalLines, 1);

        for (Pair<Point, Point> line: horizontalLines) {
            horizontalLines.set(horizontalLines.indexOf(line),extendLine(line, 0));
        }

        for (Pair<Point, Point> line: verticalLines) {
            verticalLines.set(verticalLines.indexOf(line),extendLine(line, 1));
        }

        Mat ret = new Mat(cdst.rows(), cdst.cols(), CvType.CV_8U);
        ret.setTo(new Scalar(0,0,0));

        for (Pair<Point,Point> line: horizontalLines){
            Imgproc.line(cdst, line.getKey(), line.getValue(), new Scalar(0, 255, 0), 3, Imgproc.LINE_AA, 0);
            Imgproc.line(ret, line.getKey(), line.getValue(), new Scalar(255, 0, 0), 3, Imgproc.LINE_AA, 0);
        }

        for (Pair<Point,Point> line: verticalLines){
            Imgproc.line(cdst, line.getKey(), line.getValue(), new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
            Imgproc.line(ret, line.getKey(), line.getValue(), new Scalar(255, 0, 0), 3, Imgproc.LINE_AA, 0);
        }

        show("secondMethod" , cdst, false);

        return new BoardLines(verticalLines,horizontalLines,ret);
    }

    public ArrayList<Pair<Point, Point>> correctLines(ArrayList<Pair<Point, Point>> array, int direction){
        if (array.size() < 6){
            return array;
        }

        ArrayList<Integer> forRemoval = new ArrayList<>();
        if (direction == 0){ //horizontalni

            for (int i = 0; i<array.size()-1; i++){
                double distance = array.get(i+1).getKey().y - array.get(i).getKey().y;
                if (i < array.size()/2 && distance < 50){
                    forRemoval.add(i);
                } else if (i >= array.size()/2 && distance <50){
                    forRemoval.add(i+1);
                }
            }
            for(int i = forRemoval.size()-1; i>=0; i--){
                array.remove(forRemoval.get(i).intValue());
            }

            for (int i = 0; i<8; i++){
                double expectedPositionY = array.get(array.size()-1-i).getKey().y - 80;
                if (array.size()-2-i < 0){
                    Point point1 = new Point(array.get(0).getKey().x, expectedPositionY);
                    Point point2 = new Point(array.get(0).getValue().x, expectedPositionY);
                    Pair<Point, Point> line = new Pair<>(point1, point2);
                    array.add(0, line);
                    i=0;
                } else {
                    if (expectedPositionY - array.get(array.size() - 2 - i).getKey().y > 30) {
                        double wantedPosition = array.get(array.size() - 2 - i).getKey().y + array.get(array.size() - 1 - i).getKey().y;
                        wantedPosition /= 2;
                        Point point1 = new Point(array.get(0).getKey().x, wantedPosition);
                        Point point2 = new Point(array.get(0).getValue().x, wantedPosition);
                        Pair<Point, Point> line = new Pair<>(point1, point2);
                        array.add(array.size()-1-i, line);
                        i = 0;
                    }
                }
            }

        } else { //vertikalni

            for (int i = 0; i<array.size()-1; i++){
                double distance = array.get(i+1).getKey().x - array.get(i).getKey().x;
                if (i < array.size()/2 && distance < 50){
                    forRemoval.add(i);
                } else if (i >= array.size()/2 && distance <50){
                    forRemoval.add(i+1);
                }
            }
            for(int i = forRemoval.size()-1; i>=0; i--){
                array.remove(forRemoval.get(i).intValue());
            }

            int middleIndex = 4;
            for (int i = 0; i<array.size(); i++){
                if (abs(array.get(i).getKey().x - 400) < 40){
                    middleIndex = i;
                }
            }

            double expectedPositionX;
            for (int i = 0; i<4; i++){ //leva strana
                expectedPositionX = (array.get(middleIndex-i).getKey().y < 500) ? (array.get(middleIndex-i).getKey().x - 80) : (array.get(middleIndex-i).getValue().x -80);
                if (middleIndex-i-1 < 0){
                    Point point1 = new Point(expectedPositionX, array.get(middleIndex).getKey().y);
                    Point point2 = new Point(expectedPositionX, array.get(middleIndex).getValue().y);
                    Pair<Point, Point> line = new Pair<>(point1, point2);
                    array.add(0, line);
                    middleIndex +=1;
                    i=0;
                }
            }

            for (int i = 0; i<4; i++){ //prava strana
                expectedPositionX = (array.get(middleIndex-i).getKey().y < 500) ? (array.get(middleIndex+i).getKey().x + 80) : (array.get(middleIndex+i).getValue().x + 80);
                if (middleIndex+i+1 > array.size()-1){
                    Point point1 = new Point(expectedPositionX, array.get(middleIndex).getKey().y);
                    Point point2 = new Point(expectedPositionX+5, array.get(middleIndex).getValue().y);
                    Pair<Point, Point> line = new Pair<>(point1, point2);
                    array.add(array.size(), line);
                    i=0;
                }
            }
        }
        return array;
    }

    public Pair<Point, Point> extendLine(Pair<Point, Point> line, int direction){ //horizontální 0-800, vertikální 100-800
        Point referencePoint = line.getValue();
        Pair<Point, Point> finalLine = new Pair<>(new Point(), new Point());

        double tx = line.getKey().x- line.getValue().x;
        double ty = line.getKey().y - line.getValue().y;
        double vecLength = sqrt(pow(tx,2)+pow(ty,2));
        tx /= vecLength;
        ty /= vecLength;
        if (direction == 0){ //horizontal
            double valY = referencePoint.x*ty;
            finalLine.getKey().x = 0;
            finalLine.getKey().y = referencePoint.y + valY;

            valY = (800-referencePoint.x)*ty;
            finalLine.getValue().x = 800;
            finalLine.getValue().y = referencePoint.y + valY;
        } else { //vertical
            double valX = referencePoint.y*tx;
            finalLine.getKey().x = referencePoint.x + valX;
            finalLine.getKey().y = 100;

            valX = (800-referencePoint.y)*tx;
            finalLine.getValue().x = referencePoint.x+valX;
            finalLine.getValue().y = 800;
        }
        return finalLine;
    }

    public ArrayList<Pair<Point, Point>> removeDuplicates(ArrayList<Pair<Point, Point>> array, int direction){
        if (direction == 0){ //horizontalni
            array.sort((l1,l2) -> {
                if (l1.getKey().y == l2.getKey().y) return 0;
                return l1.getKey().y > l2.getKey().y ? 1 : -1;
            });
            ArrayList<Integer> forRemoval = new ArrayList<>();

            for (int i = 0; i<array.size()-1; i++){
                if (abs(array.get(i).getKey().y - array.get(i+1).getKey().y) < 10){
                    forRemoval.add(i+1);
                }
            }
            for(int i = forRemoval.size()-1; i>=0; i--){
                array.remove(forRemoval.get(i).intValue());
            }
        } else { //vertikalni
            array.sort((l1,l2) -> {
                if (l1.getKey().x == l2.getKey().x) return 0;
                return l1.getKey().x > l2.getKey().x ? 1 : -1;
            });
            ArrayList<Integer> forRemoval = new ArrayList<>();
            for (int i = 0; i<array.size()-1; i++){
                if (abs(array.get(i).getKey().x - array.get(i+1).getKey().x) < 10){
                    forRemoval.add(i+1);
                }
            }
            for(int i = forRemoval.size()-1; i>=0; i--){
                array.remove(forRemoval.get(i).intValue());
            }
        }
        return array;
    }
}
