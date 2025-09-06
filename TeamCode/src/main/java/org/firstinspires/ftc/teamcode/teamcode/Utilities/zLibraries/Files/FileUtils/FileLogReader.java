package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Files.FileUtils;

import android.os.Environment;

import java.io.FileReader;
import java.io.IOException;

public class FileLogReader {

    private static final String BASE_FOLDER_NAME = "FIRST";

    private String fileName;
    private FileReader fileReader;
    char[] data = new char[2048];

    public FileLogReader(String fileName){
        this.fileName = fileName;
    }


    /**
     * Returns first 2048 characters in a file
     * @return
     */
    public String readFile() {
        try{
            fileReader.read(data);
        }catch (IOException e) {
            throw new Error(e);
        }
        return new String(data);
    }

    public void openFile() {
        try {

            String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;

            fileReader = new FileReader(directoryPath+"/"+fileName+".csv");

        } catch (IOException e) {
            throw new Error(e);
        }
    }

    public void closeFile() {
        try{
            fileReader.close();
        } catch (IOException e) {
            throw new Error(e);
        }
    }

}
