namespace WindowsFormsApp1
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.Com = new System.IO.Ports.SerialPort(this.components);
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.buttonConnect = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.chonCOM = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.bar1 = new System.Windows.Forms.TrackBar();
            this.bar2 = new System.Windows.Forms.TrackBar();
            this.bar3 = new System.Windows.Forms.TrackBar();
            this.bar4 = new System.Windows.Forms.TrackBar();
            this.bar5 = new System.Windows.Forms.TrackBar();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.vl6 = new System.Windows.Forms.TextBox();
            this.vl5 = new System.Windows.Forms.TextBox();
            this.vl4 = new System.Windows.Forms.TextBox();
            this.bar6 = new System.Windows.Forms.TrackBar();
            this.vl3 = new System.Windows.Forms.TextBox();
            this.vl2 = new System.Windows.Forms.TextBox();
            this.vl1 = new System.Windows.Forms.TextBox();
            this.buttonSend = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.timer2 = new System.Windows.Forms.Timer(this.components);
            this.button2 = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.bar1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar4)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar5)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar6)).BeginInit();
            this.SuspendLayout();
            // 
            // Com
            // 
            this.Com.BaudRate = 115200;
            this.Com.PortName = "COM4";
            // 
            // timer1
            // 
            this.timer1.Interval = 3000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // buttonConnect
            // 
            this.buttonConnect.BackColor = System.Drawing.Color.Lime;
            this.buttonConnect.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonConnect.ForeColor = System.Drawing.Color.RoyalBlue;
            this.buttonConnect.Location = new System.Drawing.Point(23, 73);
            this.buttonConnect.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.buttonConnect.Name = "buttonConnect";
            this.buttonConnect.Size = new System.Drawing.Size(206, 40);
            this.buttonConnect.TabIndex = 8;
            this.buttonConnect.Text = "Connect";
            this.buttonConnect.UseVisualStyleBackColor = false;
            this.buttonConnect.Click += new System.EventHandler(this.buttonConnect_Click);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.chonCOM);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.buttonConnect);
            this.groupBox1.Location = new System.Drawing.Point(12, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(255, 133);
            this.groupBox1.TabIndex = 9;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Connect";
            // 
            // chonCOM
            // 
            this.chonCOM.FormattingEnabled = true;
            this.chonCOM.Location = new System.Drawing.Point(129, 41);
            this.chonCOM.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.chonCOM.Name = "chonCOM";
            this.chonCOM.Size = new System.Drawing.Size(100, 24);
            this.chonCOM.TabIndex = 10;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(32, 45);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(41, 17);
            this.label1.TabIndex = 9;
            this.label1.Text = "Cổng";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.bar1);
            this.groupBox2.Controls.Add(this.bar2);
            this.groupBox2.Controls.Add(this.bar3);
            this.groupBox2.Controls.Add(this.bar4);
            this.groupBox2.Controls.Add(this.bar5);
            this.groupBox2.Controls.Add(this.label5);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Controls.Add(this.label4);
            this.groupBox2.Controls.Add(this.label3);
            this.groupBox2.Controls.Add(this.label2);
            this.groupBox2.Controls.Add(this.vl6);
            this.groupBox2.Controls.Add(this.vl5);
            this.groupBox2.Controls.Add(this.vl4);
            this.groupBox2.Controls.Add(this.bar6);
            this.groupBox2.Controls.Add(this.vl3);
            this.groupBox2.Controls.Add(this.vl2);
            this.groupBox2.Controls.Add(this.vl1);
            this.groupBox2.Location = new System.Drawing.Point(15, 152);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(678, 265);
            this.groupBox2.TabIndex = 11;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Control";
            // 
            // bar1
            // 
            this.bar1.LargeChange = 10;
            this.bar1.Location = new System.Drawing.Point(93, 52);
            this.bar1.Maximum = 180;
            this.bar1.Name = "bar1";
            this.bar1.Size = new System.Drawing.Size(182, 56);
            this.bar1.TabIndex = 34;
            this.bar1.Scroll += new System.EventHandler(this.bar1_Scroll);
            // 
            // bar2
            // 
            this.bar2.LargeChange = 10;
            this.bar2.Location = new System.Drawing.Point(93, 114);
            this.bar2.Maximum = 180;
            this.bar2.Name = "bar2";
            this.bar2.Size = new System.Drawing.Size(182, 56);
            this.bar2.TabIndex = 33;
            this.bar2.Scroll += new System.EventHandler(this.bar2_Scroll);
            // 
            // bar3
            // 
            this.bar3.LargeChange = 10;
            this.bar3.Location = new System.Drawing.Point(93, 176);
            this.bar3.Maximum = 180;
            this.bar3.Name = "bar3";
            this.bar3.Size = new System.Drawing.Size(182, 56);
            this.bar3.TabIndex = 32;
            this.bar3.Scroll += new System.EventHandler(this.bar3_Scroll);
            // 
            // bar4
            // 
            this.bar4.LargeChange = 10;
            this.bar4.Location = new System.Drawing.Point(441, 52);
            this.bar4.Maximum = 180;
            this.bar4.Name = "bar4";
            this.bar4.Size = new System.Drawing.Size(182, 56);
            this.bar4.TabIndex = 31;
            this.bar4.Scroll += new System.EventHandler(this.bar4_Scroll);
            // 
            // bar5
            // 
            this.bar5.LargeChange = 10;
            this.bar5.Location = new System.Drawing.Point(441, 114);
            this.bar5.Maximum = 180;
            this.bar5.Name = "bar5";
            this.bar5.Size = new System.Drawing.Size(182, 56);
            this.bar5.TabIndex = 30;
            this.bar5.Scroll += new System.EventHandler(this.bar5_Scroll);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(365, 179);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(33, 17);
            this.label5.TabIndex = 29;
            this.label5.Text = "Kẹp";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(365, 117);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(50, 17);
            this.label6.TabIndex = 28;
            this.label6.Text = "Joint 5";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(365, 55);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(50, 17);
            this.label7.TabIndex = 27;
            this.label7.Text = "Joint 4";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(17, 179);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(50, 17);
            this.label4.TabIndex = 26;
            this.label4.Text = "Joint 3";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(17, 117);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(50, 17);
            this.label3.TabIndex = 25;
            this.label3.Text = "Joint 2";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(17, 55);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(50, 17);
            this.label2.TabIndex = 24;
            this.label2.Text = "Joint 1";
            this.label2.Click += new System.EventHandler(this.label2_Click);
            // 
            // vl6
            // 
            this.vl6.Location = new System.Drawing.Point(629, 176);
            this.vl6.Name = "vl6";
            this.vl6.Size = new System.Drawing.Size(31, 22);
            this.vl6.TabIndex = 23;
            this.vl6.Text = "0";
            this.vl6.TextChanged += new System.EventHandler(this.vl6_TextChanged);
            // 
            // vl5
            // 
            this.vl5.Location = new System.Drawing.Point(629, 114);
            this.vl5.Name = "vl5";
            this.vl5.Size = new System.Drawing.Size(31, 22);
            this.vl5.TabIndex = 22;
            this.vl5.Text = "0";
            this.vl5.TextChanged += new System.EventHandler(this.vl5_TextChanged);
            // 
            // vl4
            // 
            this.vl4.Location = new System.Drawing.Point(629, 52);
            this.vl4.Name = "vl4";
            this.vl4.Size = new System.Drawing.Size(31, 22);
            this.vl4.TabIndex = 21;
            this.vl4.Text = "0";
            this.vl4.TextChanged += new System.EventHandler(this.vl4_TextChanged);
            // 
            // bar6
            // 
            this.bar6.LargeChange = 10;
            this.bar6.Location = new System.Drawing.Point(441, 176);
            this.bar6.Maximum = 180;
            this.bar6.Name = "bar6";
            this.bar6.Size = new System.Drawing.Size(182, 56);
            this.bar6.TabIndex = 20;
            this.bar6.Scroll += new System.EventHandler(this.bar6_Scroll);
            // 
            // vl3
            // 
            this.vl3.Location = new System.Drawing.Point(281, 176);
            this.vl3.Name = "vl3";
            this.vl3.Size = new System.Drawing.Size(31, 22);
            this.vl3.TabIndex = 17;
            this.vl3.Text = "0";
            this.vl3.TextChanged += new System.EventHandler(this.vl3_TextChanged);
            // 
            // vl2
            // 
            this.vl2.Location = new System.Drawing.Point(281, 114);
            this.vl2.Name = "vl2";
            this.vl2.Size = new System.Drawing.Size(31, 22);
            this.vl2.TabIndex = 16;
            this.vl2.Text = "0";
            this.vl2.TextChanged += new System.EventHandler(this.vl2_TextChanged);
            // 
            // vl1
            // 
            this.vl1.Location = new System.Drawing.Point(281, 52);
            this.vl1.Name = "vl1";
            this.vl1.Size = new System.Drawing.Size(31, 22);
            this.vl1.TabIndex = 15;
            this.vl1.Text = "0";
            this.vl1.TextChanged += new System.EventHandler(this.vl1_TextChanged);
            // 
            // buttonSend
            // 
            this.buttonSend.BackColor = System.Drawing.Color.Red;
            this.buttonSend.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.buttonSend.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.buttonSend.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonSend.ForeColor = System.Drawing.Color.GhostWhite;
            this.buttonSend.Location = new System.Drawing.Point(541, 35);
            this.buttonSend.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.buttonSend.Name = "buttonSend";
            this.buttonSend.Size = new System.Drawing.Size(97, 39);
            this.buttonSend.TabIndex = 12;
            this.buttonSend.Text = "Send";
            this.buttonSend.UseVisualStyleBackColor = false;
            this.buttonSend.Click += new System.EventHandler(this.buttonSend_Click);
            // 
            // button1
            // 
            this.button1.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.button1.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button1.ForeColor = System.Drawing.Color.Blue;
            this.button1.Location = new System.Drawing.Point(541, 86);
            this.button1.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(97, 39);
            this.button1.TabIndex = 13;
            this.button1.Text = "Home";
            this.button1.UseVisualStyleBackColor = false;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // timer2
            // 
            this.timer2.Interval = 10;
            this.timer2.Tick += new System.EventHandler(this.timer2_Tick);
            // 
            // button2
            // 
            this.button2.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.button2.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.button2.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button2.ForeColor = System.Drawing.Color.Blue;
            this.button2.Location = new System.Drawing.Point(374, 85);
            this.button2.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(97, 39);
            this.button2.TabIndex = 14;
            this.button2.Text = "Home";
            this.button2.UseVisualStyleBackColor = false;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(705, 429);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.buttonSend);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Name = "Form1";
            this.Text = "Form1";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.bar1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar5)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.bar6)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.IO.Ports.SerialPort Com;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Button buttonConnect;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.ComboBox chonCOM;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox vl6;
        private System.Windows.Forms.TextBox vl5;
        private System.Windows.Forms.TextBox vl4;
        private System.Windows.Forms.TrackBar bar6;
        private System.Windows.Forms.TextBox vl3;
        private System.Windows.Forms.TextBox vl2;
        private System.Windows.Forms.TextBox vl1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TrackBar bar1;
        private System.Windows.Forms.TrackBar bar2;
        private System.Windows.Forms.TrackBar bar3;
        private System.Windows.Forms.TrackBar bar4;
        private System.Windows.Forms.TrackBar bar5;
        private System.Windows.Forms.Button buttonSend;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Timer timer2;
        private System.Windows.Forms.Button button2;
    }
}

