package org.firstinspires.ftc.teamcode.classes.extra;

import java.io.BufferedReader;
import java.io.File;
import  java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class GethPath {

    public Node[] getpath(File Path)
            throws IOException
    {
        //creates the list used to get the path
        List<Node> Nodes = new ArrayList<Node>();

        //creates the file reader to geth the path
        BufferedReader bf = new BufferedReader(new FileReader(Path));

        String line = bf.readLine();

        //checks if the file is not empty
        while (line != null) {
         String[] dat = line.split("_", 4); // splits the line into sections for parsing
         Node cNode = new Node(Float.parseFloat(dat[0]), Float.parseFloat(dat[1]),Float.parseFloat(dat[2]) , Boolean.parseBoolean(dat[3]), Float.parseFloat(dat[4]));
         Nodes.add(cNode);
         line = bf.readLine();
        }
        bf.close(); //closes the reader if the whole file is read

        return Nodes.toArray(new Node[0]);

    }

}
