package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Files.FileUtils;

import android.os.Environment;

import java.io.FileWriter;
import java.io.IOException;

public class FileLogWriter implements LogWriter {

    private static final String BASE_FOLDER_NAME = "FIRST";

    private String fileName;
    private boolean writing = false;
    private FileWriter fileWriter;

    public FileLogWriter(String fileName) {
        this.fileName = fileName;
    }

    @Override
    public void writeLine(String line) {
        if (!writing) {
            openFile();
            writing = true;
        }

        try {
            fileWriter.append(line);
            fileWriter.append("\n");
            fileWriter.flush();
        } catch (IOException e) {
            throw new Error(e);
        }

    }

    private void openFile() {
        try {

            /*
            String logDir = System.getProperty("user.home");
            File file = new File(logDir, fileName);
            if (!file.exists()) {
                file.createNewFile();
            }
            fileWriter = new FileWriter(file);
             */

            String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;

            fileWriter = new FileWriter(directoryPath+"/"+fileName+".csv", true);
            fileWriter.flush();

        } catch (IOException e) {
            throw new Error(e);
        }
    }

    public void deleteFile(){
        try {
            String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;

            fileWriter = new FileWriter(directoryPath+"/"+fileName+".csv", false);
            fileWriter.flush();
            fileWriter.append("");
            fileWriter.flush();

        } catch (IOException e) {
            throw new Error(e);
        }
    }

    private void closeFile() {
        writing = false;
        try{
            fileWriter.close();
        } catch (IOException e) {
            throw new Error(e);
        }
    }

    @Override
    public void stop() {
        if (writing) {
            closeFile();
        }
    }

    @Override
    public boolean isWriting() {
        return writing;
    }
}

