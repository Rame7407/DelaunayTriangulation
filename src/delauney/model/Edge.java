package delauney.model;

import delauney.interf.*;

public class Edge implements IEdge {
    private IPoint P;
    private IPoint Q;
    private int Index;

    public Edge(int e, IPoint p, IPoint q) {
        Index = e;
        P = p;
        Q = q;
    }

    @Override
    public IPoint getP() {
        return P;
    }

    @Override
    public IPoint getQ() {
        return Q;
    }

    @Override
    public int getIndex() {
        return Index;
    }
}