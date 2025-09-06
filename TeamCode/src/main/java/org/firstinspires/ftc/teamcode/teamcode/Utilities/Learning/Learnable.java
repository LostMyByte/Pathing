package org.firstinspires.ftc.teamcode.teamcode.Utilities.Learning;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;

public abstract class Learnable {


    public double lr = 0.01;

    private int size;

    private Vector[] agents;

    private int numAgents;

    public Vector bestAgent;

    public abstract double fitness(Vector parameters);

    public abstract void update();

    public void train(Vector start, int numAgents, int generations) {
        size = start.length();
        this.numAgents = numAgents;
        agents = new Vector[numAgents];

        BaseOpMode.addData("Starting Tuning", "");


        for (int i = 0; i < numAgents; i++) {
            agents[i] = start.added(Vector.random(size, -lr, lr));
        }

        for (int gen = 0; gen< generations; gen++) {
            update();

            double[] fit = testAgents();
            bestAgent = agents[0];
            double bestFit = fit[0];
            for (int i =1; i < numAgents; i++) {
                if (fit[i] < bestFit) {
                    bestFit = fit[i];
                    bestAgent = agents[i];
                }
            }
            for (int i = 0; i<numAgents; i++) {
                agents[i] = newAgent(bestAgent, bestFit);
            }

        }

    }

    private double[] testAgents() {
        double[] results= new double[numAgents];
        for (int ag = 0; ag < numAgents; ag++) {

            results[ag] = fitness(agents[ag]);
            BaseOpMode.addData("Agent has fitness", results[ag]);
        }

        return results;
    }

    private Vector newAgent(Vector agent, double fitness) {

        return agent.added(Vector.random(size, 0, lr*fitness));
    }
}
