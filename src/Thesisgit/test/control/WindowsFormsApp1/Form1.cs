using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;

namespace WindowsFormsApp1
{
	public partial class Form1 : Form
	{
		int tickstart = 0;
		string ReceiveData = string.Empty;
		string DataR = string.Empty;
		delegate void SetTextCallBack();
		public Form1()
		{
			InitializeComponent();
		}

		private void label2_Click(object sender, EventArgs e)
		{

		}

		private void buttonConnect_Click(object sender, EventArgs e)
		{
			if (chonCOM.Text == "")
			{
				MessageBox.Show("Vui long chon cong COM", "Thong Bao", MessageBoxButtons.OK, MessageBoxIcon.Error);
				return;
			}
			if (buttonConnect.Text == "Connect")
			{
				try
				{
					Com.PortName = chonCOM.Text;
					Com.BaudRate = 115200;
					Com.Open();
					buttonConnect.Text = "Disconnect";
					buttonConnect.BackColor = Color.Red;
					buttonConnect.ForeColor = Color.White;
					chonCOM.Enabled = false;
					//Com.WriteLine("4444444444444444444444444444444444444444444444444444444444444");

				}
				catch (Exception)
				{
					MessageBox.Show("Không thể kết nối", "Thông Báo", MessageBoxButtons.OK, MessageBoxIcon.Error);
				}
			}
			else
			{
				while (Com.BytesToWrite > 0) { }
				Com.Dispose();
				Com.Close();
				buttonConnect.Text = "Connect";
				buttonConnect.BackColor = Color.Lime;
				buttonConnect.ForeColor = Color.RoyalBlue;

				chonCOM.Enabled = true;
			}
			tickstart = Environment.TickCount;

		}

		private void Form1_Load(object sender, EventArgs e)
		{
			
			string[] Danhsachcom = SerialPort.GetPortNames();
			Array.Sort(Danhsachcom);
			chonCOM.Items.AddRange(Danhsachcom);
			Com.DataReceived += new SerialDataReceivedEventHandler(DataReceive);
		}
		private void DataReceive(object sender, SerialDataReceivedEventArgs e)
		{
			
			SerialPort Com = (SerialPort)sender;
			ReceiveData = Com.ReadExisting();
			//Com.ReadTo(ReceiveData);
			Console.WriteLine("Data Received:");
			Console.WriteLine(ReceiveData);
			
		}

		private void bar6_Scroll(object sender, EventArgs e)
		{
			vl6.Text = bar6.Value.ToString();
			Demo();
		}

		private void bar5_Scroll(object sender, EventArgs e)
		{
			vl5.Text = bar5.Value.ToString();
			Demo();
		}

		private void bar4_Scroll(object sender, EventArgs e)
		{
			vl4.Text = bar4.Value.ToString();
			Demo();
		}

		private void bar3_Scroll(object sender, EventArgs e)
		{
			vl3.Text = bar3.Value.ToString();
			Demo();
		}

		private void bar2_Scroll(object sender, EventArgs e)
		{
			vl2.Text = bar2.Value.ToString();
			Demo();
		}

		private void bar1_Scroll(object sender, EventArgs e)
		{
			vl1.Text = bar1.Value.ToString();
			Demo();
		}

		private void vl1_TextChanged(object sender, EventArgs e)
		{
			bar1.Value = int.Parse(vl1.Text);
		}

		private void vl2_TextChanged(object sender, EventArgs e)
		{
			bar2.Value = int.Parse(vl2.Text);
		}

		private void vl3_TextChanged(object sender, EventArgs e)
		{
			bar3.Value = int.Parse(vl3.Text);
		}

		private void vl4_TextChanged(object sender, EventArgs e)
		{
			bar4.Value = int.Parse(vl4.Text);
		}

		private void vl5_TextChanged(object sender, EventArgs e)
		{
			bar5.Value = int.Parse(vl5.Text);
		}

		private void vl6_TextChanged(object sender, EventArgs e)
		{
			bar6.Value = int.Parse(vl6.Text);
		}

		private void button1_Click(object sender, EventArgs e)
		{
			vl1.Text = "0";
			vl2.Text = "0";
			vl3.Text = "90";
			vl4.Text = "130";
			vl5.Text = "30";
			vl6.Text = "0";

		}

		


		private void buttonSend_Click(object sender, EventArgs e)
		{
			Demo();
		}
		private void Demo()
		{
			this.Invoke(new MethodInvoker(delegate ()
			{
				if (Com.IsOpen)
				{
					Com.Write("S ");
					string vlbar6 = addzero(bar6.Value.ToString());
					string vlbar5 = addzero(bar5.Value.ToString());
					string vlbar4 = addzero(bar4.Value.ToString());
					string vlbar3 = addzero(bar3.Value.ToString());
					string vlbar2 = addzero(bar2.Value.ToString());
					string vlbar1 = addzero(bar1.Value.ToString());
					Com.Write(vlbar6 + " ");
					Com.Write(vlbar5 + " ");
					Com.Write(vlbar4 + " ");
					Com.Write(vlbar3 + " ");
					Com.Write(vlbar2 + " ");
					Com.Write(vlbar1 + " ");
					Com.Write("E");
				}
				else
				{
					MessageBox.Show("Chưa mở Com", "Thông Báo", MessageBoxButtons.OK, MessageBoxIcon.Warning);
					return;
				}
				timer1.Start();
				timer2.Start();
			}));
		}
		private string addzero(string a)
		{
			if(a.Length == 1) { a = "00" + a; }
			else if (a.Length == 2) { a = "0" + a; }
			return a;
		}

		private void timer1_Tick(object sender, EventArgs e)
		{
			if (ReceiveData == "ok") { timer1.Stop(); Console.WriteLine("ok"); }
			else { Demo(); Console.WriteLine("Resend"); }
			

		}

		private void timer2_Tick(object sender, EventArgs e)
		{
			
		}

		private void button2_Click(object sender, EventArgs e)
		{
			vl1.Text = "0";
			vl2.Text = "0";
			vl3.Text = "50";
			vl4.Text = "100";
			vl5.Text = "80";
			vl6.Text = "40";
		}
	}
}
