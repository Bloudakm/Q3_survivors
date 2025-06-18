using System;

public class Node1
{
    public int X, Y;
    public bool Walkable;
    public int G, H;
    public Node1 Parent;
    public int F => G + H;

    public Node1(int x, int y, bool walkable = true)
    {
        X = x; Y = y;
        Walkable = walkable;
        G = H = 0;
        Parent = null;
    }

    public override bool Equals(object obj) => obj is Node1 n && X == n.X && Y == n.Y;
    public override int GetHashCode() => HashCode.Combine(X, Y);
}