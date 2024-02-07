package math;

public class QuadraticEquation {
	double a, b, c;

	public QuadraticEquation withA(double a) {
		this.a = a;
		return this;
	}

	public QuadraticEquation withB(double b) {
		this.b = b;
		return this;
	}

	public QuadraticEquation withC(double c) {
		this.c = c;
		return this;
	}

	public double get(double input) {
		return a * Math.pow(input, 2) + b * input + c;
	}

}
