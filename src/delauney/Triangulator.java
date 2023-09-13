package delauney;

import delauney.interf.*;
import delauney.model.*;

import java.util.*;


public class Triangulator {
    private final double epsilon = Math.pow(2, -52);

    // Это массив, используемый во время операции легализации ребер.
    // Он служит в качестве стека для хранения индексов полуребер, которые требуют дальнейшей обработки.
    private final int[] edgeStack = new int[512];

    // Массив, который хранит индексы точек, образующих треугольники. Каждые три последовательных элемента в массиве представляют один треугольник.
    private  int[] triangles;
    // Массив, который хранит индексы полуребер. Каждое полуребро связывает две точки и указывает на индекс соседнего полуребра.
    private  int[] halfedges;
    private final IPoint[] points;
    // Массив, который хранит индексы точек, образующих выпуклую оболочку. Эти точки образуют внешний контур триангуляции.
    private final int[] hull;
    private final int hashSize;
    // Массивы, используемые для хранения информации о выпуклой оболочке. Они содержат индексы предыдущего и следующего полуребер,
    // а также индексы треугольников и хэш-таблицу для быстрого доступа к полуребрам выпуклой оболочки.
    private  int[] hullPrev;
    private  int[] hullNext;
    private  int[] hullTri;
    private final int[] hullHash;
    // Координаты центра триангуляции, которые используются для вычисления ориентации точек и определения выпуклой оболочки.
    private  double centerX;
    private  double centerY;
    private int trianglesLen;
    private final double[] coords;
    private  int hullStart;
    private  int hullSize = 3;

    public int[] getTriangles() {

        return triangles;
    }
    public double[] getCoords() {

        return coords;
    }


    public Triangulator(IPoint[] points) throws Exception {


        if (points.length < 3)
            throw new IllegalArgumentException("Need at least 3 points");

        this.points = points;
        this.coords = new double[points.length * 2];

        for (int i = 0; i < points.length; i++) {
            IPoint p = points[i];
            this.coords[2 * i] = p.getX();
            this.coords[2 * i + 1] = p.getY();
        }

        int pointsCount = points.length;
        int maxTriangles = 2 * pointsCount - 5;

        this.triangles = new int[maxTriangles * 3];

        this.halfedges = new int[maxTriangles * 3];
        this.hashSize = (int) Math.ceil(Math.sqrt(pointsCount));

        this.hullPrev = new int[pointsCount];
        this.hullNext = new int[pointsCount];
        this.hullTri = new int[pointsCount];
        this.hullHash = new int[hashSize];

        int[] ids = new int[pointsCount];

        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < pointsCount; i++) {
            double x = coords[2 * i];
            double y = coords[2 * i + 1];
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            ids[i] = i;
        }

        double centerX = (minX + maxX) / 2;
        double centerY = (minY + maxY) / 2;

        double minDist = Double.POSITIVE_INFINITY;
        int i0 = 0;
        int i1 = 0;
        int i2 = 0;

        // выбираем начальную (i0) ближайшую точку к центру триангуляции
        for (int i = 0; i < pointsCount; i++) {
            double d = dist(centerX, centerY, coords[2 * i], coords[2 * i + 1]);
            if (d < minDist) {
                i0 = i;
                minDist = d;
            }
        }

        double i0x = coords[2 * i0];
        double i0y = coords[2 * i0 + 1];

        minDist = Double.POSITIVE_INFINITY;

        // находим вторую точку
        for (int i = 0; i < pointsCount; i++) {
            if (i == i0) continue;
            double d = dist(i0x, i0y, coords[2 * i], coords[2 * i + 1]);
            if (d < minDist && d > 0) {
                i1 = i;
                minDist = d;
            }
        }

        double i1x = coords[2 * i1];
        double i1y = coords[2 * i1 + 1];

        double minRadius = Double.POSITIVE_INFINITY;

        // и находим третью; эти точки образуют наименьший описывающий окружности треугольник с первой точкой.
        for (int i = 0; i < pointsCount; i++) {
            if (i == i0 || i == i1) continue;
            double r = circumradius(i0x, i0y, i1x, i1y, coords[2 * i], coords[2 * i + 1]);
            if (r < minRadius) {
                i2 = i;
                minRadius = r;
            }
        }

        double i2x = coords[2 * i2];
        double i2y = coords[2 * i2 + 1];

        // Проверяем, существует ли такой треугольник.
        if (Double.isInfinite(minRadius))
            throw new Exception("No Delaunay triangulation exists for this input.");

        if (orient(i0x, i0y, i1x, i1y, i2x, i2y)) {
            int i = i1;
            double x = i1x;
            double y = i1y;
            i1 = i2;
            i1x = i2x;
            i1y = i2y;
            i2 = i;
            i2x = x;
            i2y = y;
        }

        Point center = circumcenter(i0x, i0y, i1x, i1y, i2x, i2y);
        centerX = center.getX();
        centerY = center.getY();

        double[] dists = new double[pointsCount];
        for (int i = 0; i < pointsCount; i++)
            dists[i] = dist(coords[2 * i], coords[2 * i + 1], center.getX(), center.getY());

    // сортируем точки по расстоянию от центра окружности начального треугольника
        quicksort(ids, dists, 0, pointsCount - 1);

        // устанавливаем начальную выпуклую оболочку, состоящую из начального треугольника
        hullStart = i0;
        hullSize = 3;

        hullNext[i0] = hullPrev[i2] = i1;
        hullNext[i1] = hullPrev[i0] = i2;
        hullNext[i2] = hullPrev[i1] = i0;

        hullTri[i0] = 0;
        hullTri[i1] = 1;
        hullTri[i2] = 2;

        hullHash[hashKey(i0x, i0y)] = i0;
        hullHash[hashKey(i1x, i1y)] = i1;
        hullHash[hashKey(i2x, i2y)] = i2;

        trianglesLen = 0;
        addTriangle(i0, i1, i2, -1, -1, -1);

        double xp = 0;
        double yp = 0;

        for (int k = 0; k < ids.length; k++) {
            int i = ids[k];
            double x = coords[2 * i];
            double y = coords[2 * i + 1];


            if (k > 0 && Math.abs(x - xp) <= epsilon && Math.abs(y - yp) <= epsilon) continue;
            xp = x;
            yp = y;

            // пропустить исходные точки треугольника
            if (i == i0 || i == i1 || i == i2) continue;

            // находим видимое ребро на выпуклой оболочке, используя хэш-таблицу ребер
            int start = 0;
            for (int j = 0; j < hashSize; j++) {
                int key = hashKey(x, y);
                start = hullHash[(key + j) % hashSize];
                if (start != -1 && start != hullNext[start]) break;
            }

            start = hullPrev[start];
            int e = start;
            int q = hullNext[e];

            while (!orient(x, y, coords[2 * e], coords[2 * e + 1], coords[2 * q], coords[2 * q + 1])) {
                e = q;
                if (e == start) {
                    e = Integer.MAX_VALUE;
                    break;
                }

                q = hullNext[e];
            }

            if (e == Integer.MAX_VALUE) continue; // likely a near-duplicate point; skip it

            // Добавляем новый треугольник, связанный с точкой
            int t = addTriangle(e, i, hullNext[e], -1, -1, hullTri[e]);

        // выполняем рекурсивное переворачивание треугольников до выполнения условия Делоне
            hullTri[i] = legalize(t + 2);
            hullTri[e] = t; // keep track of boundary triangles on the hull
            hullSize++;

            int next = hullNext[e];
            q = hullNext[next];

            while (orient(x, y, coords[2 * next], coords[2 * next + 1], coords[2 * q], coords[2 * q + 1])) {
                t = addTriangle(next, i, q, hullTri[i], -1, hullTri[next]);
                hullTri[i] = legalize(t + 2);
                hullNext[next] = next; // mark as removed
                hullSize--;
                next = q;

                q = hullNext[next];
            }

            if (e == start) {
                q = hullPrev[e];

                while (orient(x, y, coords[2 * q], coords[2 * q + 1], coords[2 * e], coords[2 * e + 1])) {
                    t = addTriangle(q, i, e, -1, hullTri[e], hullTri[q]);
                    legalize(t + 2);
                    hullTri[q] = t;
                    hullNext[e] = e; // отметить как удаленное
                    hullSize--;
                    e = q;

                    q = hullPrev[e];
                }
            }
            // обновляем индексы выпуклой оболочки
            hullStart = hullPrev[i] = e;
            hullNext[e] = hullPrev[next] = i;
            hullNext[i] = next;

            // и хэш-таблицу ребер
            hullHash[hashKey(x, y)] = i;
            hullHash[hashKey(coords[2 * e], coords[2 * e + 1])] = e;
        }
        // Формируем массив Hull, содержащий индексы точек выпуклой оболочки.
        hull = new int[hullSize];
        int s = hullStart;
        for (int i = 0; i < hullSize; i++) {
            hull[i] = s;
            s = hullNext[s];
        }

        // Удаляем лишние массивы
        hullPrev = hullNext = hullTri = null;

        // Обрезаем массивы Triangles и Halfedges, чтобы они содержали только треугольники, созданные во время триангуляции.
        triangles = Arrays.copyOf(triangles, trianglesLen * 3);
        halfedges = Arrays.copyOf(halfedges, trianglesLen * 3);

    }

    /**
     * Выполняет операцию "легализации" ребра в триангуляции. Эта операция гарантирует, что ребро удовлетворяет
     * условию Делоне, то есть точка p1 не находится в окружности, описанной вокруг треугольника,
     * образованного точками p0, pl и pr. Если ребро не удовлетворяет условию Делоне, оно "переворачивается" путем замены точек p0 и p1 местами.
     */
    private int legalize(int a) {
        int i = 0;
        int ar;

        // рекурсия устраняется благодаря стеку фиксированного размера edgeStack
        while (true) {
            // получаем связанное с a полуребро b
            int b = halfedges[a];

            int a0 = a - a % 3;
            ar = a0 + (a + 2) % 3;

            if (b == -1) {
                if (i == 0) break;
                a = edgeStack[--i];
                continue;
            }

            int b0 = b - b % 3;
            int al = a0 + (a + 1) % 3;
            int bl = b0 + (b + 2) % 3;

            // находим индексы точек треугольника p
            int p0 = triangles[ar];
            int pr = triangles[a];
            int pl = triangles[al];
            int p1 = triangles[bl];

            // проверка "внутри окружности?"
            boolean illegal = inCircle(
                    coords[2 * p0], coords[2 * p0 + 1],
                    coords[2 * pr], coords[2 * pr + 1],
                    coords[2 * pl], coords[2 * pl + 1],
                    coords[2 * p1], coords[2 * p1 + 1]);

            if (illegal) {
                // если внутри, то делаем свап p0 и p1
                triangles[a] = p1;
                triangles[b] = p0;

                int hbl = halfedges[bl];
                if (hbl == -1) {
                    int e = hullStart;
                    do {
                        if (hullTri[e] == bl) {
                            hullTri[e] = a;
                            break;
                        }

                        e = hullPrev[e];
                    } while (e != hullStart);
                }

                // Обновляем ссылки на полуребра, связанные с ребром.
                // Если полуребро bl (связанное с ребром на другой стороне оболочки) не имеет ссылки на полуребро,
                // метод выполняет поиск полуребра bl в оболочке и обновляет его ссылку на a.
                link(a, hbl);
                link(b, halfedges[ar]);
                link(ar, bl);

                int br = b0 + (b + 1) % 3;

                // Добавляет индекс полуребра br (связанного с ребром на другой стороне) в стек edgeStack для последующей обработки.
                if (i < edgeStack.length) {
                    edgeStack[i++] = br;
                }
            } else {
                // проверяем, есть ли ещё индексы полуребер в стеке, если нет - процесс легализации пройден
                if (i == 0) break;
                a = edgeStack[--i];
            }
        }

        return ar;
    }

    /**
     * Для определения, находится ли точка P внутри окружности, описанной вокруг треугольника с вершинами A, B и C.
     * Метод возвращает true, если точка P находится внутри окружности, и false в противном случае.
     */
    private static boolean inCircle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py) {
        double dx = ax - px;
        double dy = ay - py;
        double ex = bx - px;
        double ey = by - py;
        double fx = cx - px;
        double fy = cy - py;

        double ap = dx * dx + dy * dy;
        double bp = ex * ex + ey * ey;
        double cp = fx * fx + fy * fy;

        return dx * (ey * cp - bp * fy) -
                dy * (ex * cp - bp * fx) +
                ap * (ex * fy - ey * fx) < 0;
    }



    /**
     * Для добавления нового треугольника в список треугольников и установки связей между треугольниками.
     * Аргументы метода i0, i1 и i2 представляют индексы вершин треугольника в списке вершин.
     * Аргументы a, b и c представляют индексы смежных треугольников.
     * @return Индекс нового треугольника t
     */
    private int addTriangle(int i0, int i1, int i2, int a, int b, int c) {
        int t = trianglesLen;

        triangles[t] = i0;
        triangles[t + 1] = i1;
        triangles[t + 2] = i2;

        link(t, a);
        link(t + 1, b);
        link(t + 2, c);

        trianglesLen += 3;
        return t;
    }

    /**
     * Используется для установки связи между двумя ребрами треугольников.
     * @param a Индекс ребра в списке полуребер (одна сторона треуг)
     * @param b Индекс ребра в списке полуребер (смежная сторона треуг)
     */
    private void link(int a, int b) {
        halfedges[a] = b;
        if (b != -1) halfedges[b] = a;
    }

    /**
     * Используется для хэширования и сортировки вершин треугольников.
     * Принимает координаты x и y вершины и возвращает хеш-ключ, который используется для индексации вершин в хэш-таблице.
     * Хеш-таблица используется для быстрого поиска ближайших соседей вершин.
     * @param x Координата X вершины
     * @param y Координата Y вершины
     * @return Хеш-ключ
     */
    private int hashKey(double x, double y) {
        return (int) (Math.floor(pseudoAngle(x - centerX, y - centerY) * hashSize) % hashSize);
    }

    /**
     * Принимает разности dx и dy координат и вычисляет псевдоугол, который используется для сортировки вершин.
     * @param diffX Разность координат X
     * @param diffY Разность координат Y
     * @return Псевдоугол
     */
    private static double pseudoAngle(double diffX, double diffY) {
        double normalizedDiffX = diffX / (Math.abs(diffX) + Math.abs(diffY));
        return (diffY > 0 ? 3 - normalizedDiffX : 1 + normalizedDiffX) / 4;
    }

    private static void quicksort(int[] ids, double[] dists, int left, int right) {
        if (right - left <= 20) {
            for (int i = left + 1; i <= right; i++) {
                int temp = ids[i];
                double tempDist = dists[temp];
                int j = i - 1;
                while (j >= left && dists[ids[j]] > tempDist) {
                    ids[j + 1] = ids[j];
                    j--;
                }
                ids[j + 1] = temp;
            }
        } else {
            int median = (left + right) >> 1;
            int i = left + 1;
            int j = right;
            swap(ids, median, i);
            if (dists[ids[left]] > dists[ids[right]]) swap(ids, left, right);
            if (dists[ids[i]] > dists[ids[right]]) swap(ids, i, right);
            if (dists[ids[left]] > dists[ids[i]]) swap(ids, left, i);

            int temp = ids[i];
            double tempDist = dists[temp];
            while (true) {
                do {
                    i++;
                } while (dists[ids[i]] < tempDist);
                do {
                    j--;
                } while (dists[ids[j]] > tempDist);
                if (j < i) break;
                swap(ids, i, j);
            }

            ids[left + 1] = ids[j];
            ids[j] = temp;

            if (right - i + 1 >= j - left) {
                quicksort(ids, dists, i, right);
                quicksort(ids, dists, left, j - 1);
            } else {
                quicksort(ids, dists, left, j - 1);
                quicksort(ids, dists, i, right);
            }
        }
    }

    private static void swap(int[] arr, int i, int j) {
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }

    private static boolean orient(double px, double py, double qx, double qy, double rx, double ry) {
        return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0;
    }

    private static double circumradius(double ax, double ay, double bx, double by, double cx, double cy) {
        double dx = bx - ax;
        double dy = by - ay;
        double ex = cx - ax;
        double ey = cy - ay;
        double bl = dx * dx + dy * dy;
        double cl = ex * ex + ey * ey;
        double d = 0.5 / (dx * ey - dy * ex);
        double x = (ey * bl - dy * cl) * d;
        double y = (dx * cl - ex * bl) * d;
        return x * x + y * y;
    }

    private static Point circumcenter(double ax, double ay, double bx, double by, double cx, double cy) {
        double dx = bx - ax;
        double dy = by - ay;
        double ex = cx - ax;
        double ey = cy - ay;
        double bl = dx * dx + dy * dy;
        double cl = ex * ex + ey * ey;
        double d = 0.5 / (dx * ey - dy * ex);
        double x = ax + (ey * bl - dy * cl) * d;
        double y = ay + (dx * cl - ex * bl) * d;

        return new Point((int) x, (int) y);
    }

    private static double dist(double ax, double ay, double bx, double by) {
        double diffX = ax - bx;
        double diffY = ay - by;
        return diffX * diffX + diffY * diffY;
    }

    public List<IEdge> getEdges() {
        List<IEdge> edges = new ArrayList<>();

        for (int e = 0; e < triangles.length; e++) {
            if (e > halfedges[e]) {
                Point p = (Point) points[triangles[e]];
                Point q = (Point)points[triangles[nextHalfedge(e)]];
                edges.add(new Edge(e, p, q));
            }
        }

        return edges;
    }

    private int nextHalfedge(int e) {
        return (e % 3 == 2) ? e - 2 : e + 1;
    }



}