package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

public class StringConstructor {

    int spacesBefore;
    String string;

    public StringConstructor(int initialSpaces){
        string = "";
        spacesBefore = initialSpaces;
    }

    public StringConstructor append(String s, int l){
        String beforeSpace = spaces.substring(0, spacesBefore);
        if(l < s.length()){
            string = string + beforeSpace + s.substring(0, l);
            spacesBefore = 0;
        }else{
            string = string + beforeSpace + s;
            spacesBefore = l-s.length();
        }

        return this;
    }

    public String toString(){
        System.out.println(string);
        return string;
    }

    String spaces = "                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ";

}
